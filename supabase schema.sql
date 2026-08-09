-- ============================================================================
--  HCHO-Guard : Formaldehyde (MS1100 + ESP32) Monitoring Schema
--  Project : veiaafecvwthgkplofog.supabase.co
--  Run this whole file in Supabase Studio -> SQL Editor -> New query -> Run
-- ============================================================================

-- ---------------------------------------------------------------------------
-- 1. DEVICES  (one row per sensor node)
-- ---------------------------------------------------------------------------
create table if not exists public.devices (
  device_id      text primary key,                 -- must match DEVICE_ID in firmware
  name           text        not null default 'Unnamed node',
  location       text,
  sensor_model   text        not null default 'MS1100',
  firmware       text,
  r0_clean_air   double precision,                 -- last calibration value (kOhm ratio base)
  warn_ppm       double precision not null default 0.08,   -- WHO 30-min guideline
  alarm_ppm      double precision not null default 0.30,
  last_seen      timestamptz,
  is_active      boolean     not null default true,
  created_at     timestamptz not null default now()
);

comment on table public.devices is 'Sensor node registry. device_id here MUST be identical to DEVICE_ID compiled into the ESP32 firmware.';

-- ---------------------------------------------------------------------------
-- 2. READINGS  (time-series, the main table)
-- ---------------------------------------------------------------------------
create table if not exists public.readings (
  id             bigint generated always as identity primary key,
  device_id      text        not null references public.devices(device_id) on delete cascade,
  recorded_at    timestamptz not null default now(),   -- time from the node (NTP)
  received_at    timestamptz not null default now(),   -- time the DB accepted the row

  -- raw sensor domain
  adc_raw        integer,                              -- 0..4095 averaged ADC counts
  voltage_mv     double precision,                     -- sensor output at the ADC pin
  rs_kohm        double precision,                     -- sensing resistance
  ratio          double precision,                     -- Rs / R0

  -- gas domain
  hcho_ppm       double precision not null,
  hcho_mg_m3     double precision,                     -- ppm * 1.228 @25C, 1 atm
  level          text        not null default 'good',  -- good | moderate | warn | alarm

  -- environment (optional DHT22 / BME280)
  temperature_c  double precision,
  humidity_pct   double precision,

  -- housekeeping
  rssi           integer,
  uptime_s       bigint,
  fw             text,
  is_calibrated  boolean     not null default true,
  constraint readings_level_chk check (level in ('good','moderate','warn','alarm')),
  constraint readings_ppm_chk   check (hcho_ppm >= 0 and hcho_ppm < 1000)
);

create index if not exists idx_readings_device_time on public.readings (device_id, recorded_at desc);
create index if not exists idx_readings_time        on public.readings (recorded_at desc);
create index if not exists idx_readings_level       on public.readings (level) where level in ('warn','alarm');

-- ---------------------------------------------------------------------------
-- 3. ALERTS  (auto-raised when a reading crosses the node threshold)
-- ---------------------------------------------------------------------------
create table if not exists public.alerts (
  id            bigint generated always as identity primary key,
  device_id     text        not null references public.devices(device_id) on delete cascade,
  reading_id    bigint      references public.readings(id) on delete set null,
  raised_at     timestamptz not null default now(),
  hcho_ppm      double precision not null,
  level         text        not null,
  message       text,
  acknowledged  boolean     not null default false
);

create index if not exists idx_alerts_device_time on public.alerts (device_id, raised_at desc);

-- ---------------------------------------------------------------------------
-- 4. TRIGGERS
-- ---------------------------------------------------------------------------

-- 4a. keep devices.last_seen fresh + stamp mg/m3 and level if the node omitted them
create or replace function public.fn_on_reading_insert()
returns trigger
language plpgsql
security definer
set search_path = public
as $$
declare
  v_warn  double precision;
  v_alarm double precision;
begin
  if new.hcho_mg_m3 is null then
    new.hcho_mg_m3 := round((new.hcho_ppm * 1.228)::numeric, 4);
  end if;

  select warn_ppm, alarm_ppm into v_warn, v_alarm
  from public.devices where device_id = new.device_id;

  v_warn  := coalesce(v_warn, 0.08);
  v_alarm := coalesce(v_alarm, 0.30);

  new.level := case
    when new.hcho_ppm >= v_alarm       then 'alarm'
    when new.hcho_ppm >= v_warn        then 'warn'
    when new.hcho_ppm >= v_warn * 0.6  then 'moderate'
    else 'good'
  end;

  return new;
end;
$$;

drop trigger if exists trg_reading_before_insert on public.readings;
create trigger trg_reading_before_insert
  before insert on public.readings
  for each row execute function public.fn_on_reading_insert();

-- 4b. after insert: touch last_seen, raise an alert row on warn/alarm
create or replace function public.fn_after_reading_insert()
returns trigger
language plpgsql
security definer
set search_path = public
as $$
declare
  v_last_level text;
begin
  update public.devices
     set last_seen = new.recorded_at,
         firmware  = coalesce(new.fw, firmware)
   where device_id = new.device_id;

  if new.level in ('warn','alarm') then
    select level into v_last_level
      from public.alerts
     where device_id = new.device_id
       and raised_at > now() - interval '10 minutes'
     order by raised_at desc limit 1;

    -- only open a new alert if we are not already inside the same alert state
    if v_last_level is distinct from new.level then
      insert into public.alerts (device_id, reading_id, hcho_ppm, level, message)
      values (new.device_id, new.id, new.hcho_ppm, new.level,
              format('%s ppm HCHO detected (%s)', round(new.hcho_ppm::numeric,3), new.level));
    end if;
  end if;

  return null;
end;
$$;

drop trigger if exists trg_reading_after_insert on public.readings;
create trigger trg_reading_after_insert
  after insert on public.readings
  for each row execute function public.fn_after_reading_insert();

-- ---------------------------------------------------------------------------
-- 5. VIEWS  (dashboard + dataset building)
-- ---------------------------------------------------------------------------

-- latest reading per device
create or replace view public.v_latest_reading as
select distinct on (device_id) *
from public.readings
order by device_id, recorded_at desc;

-- minute buckets (good for charting a long window without pulling every row)
create or replace view public.v_readings_minutely as
select
  device_id,
  date_trunc('minute', recorded_at)          as bucket,
  count(*)                                   as samples,
  round(avg(hcho_ppm)::numeric, 4)           as ppm_avg,
  round(min(hcho_ppm)::numeric, 4)           as ppm_min,
  round(max(hcho_ppm)::numeric, 4)           as ppm_max,
  round(avg(temperature_c)::numeric, 2)      as temp_avg,
  round(avg(humidity_pct)::numeric, 2)       as rh_avg
from public.readings
group by device_id, date_trunc('minute', recorded_at);

-- hourly buckets
create or replace view public.v_readings_hourly as
select
  device_id,
  date_trunc('hour', recorded_at)            as bucket,
  count(*)                                   as samples,
  round(avg(hcho_ppm)::numeric, 4)           as ppm_avg,
  round(min(hcho_ppm)::numeric, 4)           as ppm_min,
  round(max(hcho_ppm)::numeric, 4)           as ppm_max,
  round(stddev_samp(hcho_ppm)::numeric, 4)   as ppm_sd,
  round(avg(temperature_c)::numeric, 2)      as temp_avg,
  round(avg(humidity_pct)::numeric, 2)       as rh_avg
from public.readings
group by device_id, date_trunc('hour', recorded_at);

-- daily rollup, includes WHO exceedance minutes
create or replace view public.v_readings_daily as
select
  device_id,
  (recorded_at at time zone 'Asia/Dhaka')::date  as day,
  count(*)                                       as samples,
  round(avg(hcho_ppm)::numeric, 4)               as ppm_avg,
  round(max(hcho_ppm)::numeric, 4)               as ppm_peak,
  round(stddev_samp(hcho_ppm)::numeric, 4)       as ppm_sd,
  count(*) filter (where level in ('warn','alarm')) as exceed_samples
from public.readings
group by device_id, (recorded_at at time zone 'Asia/Dhaka')::date;

-- ---------------------------------------------------------------------------
-- 6. RPC : flexible stats for the dataset builder
-- ---------------------------------------------------------------------------
create or replace function public.get_reading_stats(
  p_device text,
  p_from   timestamptz,
  p_to     timestamptz
)
returns table (
  samples     bigint,
  ppm_avg     double precision,
  ppm_sd      double precision,
  ppm_min     double precision,
  ppm_max     double precision,
  ppm_p95     double precision,
  exceed_pct  double precision
)
language sql
stable
as $$
  select
    count(*)                                                              as samples,
    avg(hcho_ppm)                                                         as ppm_avg,
    stddev_samp(hcho_ppm)                                                 as ppm_sd,
    min(hcho_ppm)                                                         as ppm_min,
    max(hcho_ppm)                                                         as ppm_max,
    percentile_cont(0.95) within group (order by hcho_ppm)                as ppm_p95,
    100.0 * count(*) filter (where level in ('warn','alarm')) / nullif(count(*),0) as exceed_pct
  from public.readings
  where device_id = p_device
    and recorded_at between p_from and p_to;
$$;

-- ---------------------------------------------------------------------------
-- 7. ROW LEVEL SECURITY
--    Demo/lab posture: the anon key may register a device, insert readings,
--    and read everything. Nothing can be updated or deleted from the client.
--    For a graded/production deployment, replace the insert policy with an
--    Edge Function holding the service_role key + a per-node shared secret.
-- ---------------------------------------------------------------------------
alter table public.devices  enable row level security;
alter table public.readings enable row level security;
alter table public.alerts   enable row level security;

drop policy if exists devices_read       on public.devices;
drop policy if exists devices_insert     on public.devices;
drop policy if exists devices_update     on public.devices;
drop policy if exists readings_read      on public.readings;
drop policy if exists readings_insert    on public.readings;
drop policy if exists alerts_read        on public.alerts;
drop policy if exists alerts_ack         on public.alerts;

create policy devices_read    on public.devices  for select using (true);
create policy devices_insert  on public.devices  for insert with check (true);
create policy devices_update  on public.devices  for update using (true) with check (true);

create policy readings_read   on public.readings for select using (true);
create policy readings_insert on public.readings for insert with check (true);

create policy alerts_read     on public.alerts   for select using (true);
create policy alerts_ack      on public.alerts   for update using (true) with check (true);

-- ---------------------------------------------------------------------------
-- 8. REALTIME  (so the dashboard updates without polling)
-- ---------------------------------------------------------------------------
do $$
begin
  if not exists (
    select 1 from pg_publication_tables
    where pubname = 'supabase_realtime' and tablename = 'readings'
  ) then
    alter publication supabase_realtime add table public.readings;
  end if;
  if not exists (
    select 1 from pg_publication_tables
    where pubname = 'supabase_realtime' and tablename = 'alerts'
  ) then
    alter publication supabase_realtime add table public.alerts;
  end if;
end $$;

-- ---------------------------------------------------------------------------
-- 9. SEED  ->  device_id MUST equal DEVICE_ID in the .ino file
-- ---------------------------------------------------------------------------
insert into public.devices (device_id, name, location, sensor_model, warn_ppm, alarm_ppm)
values ('hcho-node-01', 'Fish Market Node', 'Savar Bazar, Dhaka', 'MS1100', 0.08, 0.30)
on conflict (device_id) do nothing;

-- Optional second node for multi-point studies
insert into public.devices (device_id, name, location, sensor_model, warn_ppm, alarm_ppm)
values ('hcho-node-02', 'Lab Reference Node', 'ESRC Lab, DIU', 'MS1100', 0.08, 0.30)
on conflict (device_id) do nothing;

-- ---------------------------------------------------------------------------
-- 10. HOUSEKEEPING (optional) : drop readings older than 180 days
--     Enable pg_cron from Database -> Extensions first, then uncomment.
-- ---------------------------------------------------------------------------
-- select cron.schedule('purge_old_readings','0 3 * * *',
--   $$delete from public.readings where recorded_at < now() - interval '180 days'$$);
