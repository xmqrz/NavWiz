<script>
  import { ProgressBar } from '@skeletonlabs/skeleton';
  import { get } from 'svelte/store';
  import { differenceInDays } from 'date-fns';
  import { serverClock } from 'stores/server-clock.js';

  const DAYS_COUNTDOWN = 30;
  const MILEAGE_COUNTDOWN = 1000;

  export let dashboard = {};

  const nextPmDue = new Date(dashboard.next_pm_due);
  const nextPmMileage = dashboard.next_pm_mileage || '';

  const [clock, _tz] = get(serverClock);

  let progress = 0;
  let days;
  let mileage;
  if (!isNaN(nextPmDue) && !isNaN(nextPmMileage) && !isNaN(dashboard.current_mileage)) {
    days = Math.max(differenceInDays(nextPmDue, clock), 0);
    mileage = Math.max(nextPmMileage - dashboard.current_mileage, 0);

    progress = Math.max(
      ((DAYS_COUNTDOWN - days) / DAYS_COUNTDOWN) * 100,
      ((MILEAGE_COUNTDOWN - mileage) / MILEAGE_COUNTDOWN) * 100
    );
    progress = Math.min(Math.max(progress, 0), 100);
    progress = Math.round(progress);
  }
</script>

<div class="card h-full w-full shadow-lg">
  <header class="card-header w-full pb-3 pt-7">
    <div class="text-center text-lg font-light tracking-widest">PREVENTIVE MAINTENANCE</div>
  </header>
  <hr />
  <section class="px-7 py-3 pt-10">
    <div class="table-container">
      <table class="table">
        <tbody>
          <tr>
            <td>Next P.M. Due</td>
            <td>{dashboard.next_pm_due || '-'}</td>
          </tr>
          <tr>
            <td>Next P.M. Mileage</td>
            <td>{isNaN(dashboard.next_pm_mileage) ? '-' : dashboard.next_pm_mileage} m</td>
          </tr>
          <tr>
            <td>Countdown</td>
            <td
              >{isNaN(mileage) ? '-' : mileage.toLocaleString()} m / {isNaN(days)
                ? '-'
                : days.toLocaleString()} days</td>
          </tr>
        </tbody>
      </table>
    </div>
  </section>
  <footer class="card-footer">
    <div class="p-5 pt-10 text-right text-xs">
      <ProgressBar label="Progress Bar" value={progress} max={100} />
      {progress} %
    </div>
  </footer>
</div>
