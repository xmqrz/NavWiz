<script>
  import SkillsetImg from 'components/SkillsetImg.svelte';
  import PMCard from './components/PMCard.svelte';
  import TaskCard from './components/TaskCard.svelte';
  import BatteryCard from './components/BatteryCard.svelte';
  import MileageCard from './components/MileageCard.svelte';
  import ActivityCard from './components/ActivityCard.svelte';
  import { whiteLabel } from 'stores/white-label.js';
  import { title } from 'stores/page-title';

  export let data;

  title.set('');
  // TODO: all card fillin data (from last stats build to now) using live data.
</script>

<section class="auto-col-min grid w-full grid-cols-1 gap-4 p-7 lg:grid-cols-3">
  <div class="card row-span-3 h-full w-full shadow-lg">
    <header class="card-header flex w-full flex-col items-center gap-4 p-7">
      <SkillsetImg src={data.dashboard.skillset_img} skillset={data.dashboard.skillset} />
      <div>
        <div class="text-center text-xl font-semibold">
          {data.agv.name || '-'}
        </div>
        <div class="text-center text-xs opacity-50">
          {data.agv.uuid || '-'}
        </div>
      </div>
    </header>
    <hr class="opacity-50" />
    <section class="flex flex-row flex-wrap justify-center gap-10 p-4">
      <div class="text-center">
        <div class="truncate text-xl font-bold text-secondary-700">
          {data.dashboard.uptime}
        </div>
        <div class="truncate text-sm tracking-widest opacity-50">Uptime</div>
      </div>
      <div class="text-center">
        <div class="truncate text-xl font-bold text-secondary-700">
          {data.agv.task_counter || '-'}
        </div>
        <div class="truncate text-sm tracking-widest opacity-50">Total Task</div>
      </div>
      <div class="text-center">
        <div class="truncate text-xl font-bold text-secondary-700">
          {data.agv.mileage ? data.agv.mileage + ' m' : '-'}
        </div>
        <div class="truncate text-sm tracking-widest opacity-50">Mileage</div>
      </div>
    </section>
    <section class="px-16 pb-7 pt-3">
      <div class="tracking-widest text-secondary-700">DETAILS</div>
      <div>
        <div class="pt-3 text-sm tracking-wider opacity-50">AGV Model</div>
        <div class="pt-1 text-lg font-semibold">{data.dashboard.agv_model}</div>
      </div>
      <div>
        <div class="pt-3 text-sm tracking-wider opacity-50">Serial Number</div>
        <div class="pt-1 text-lg font-semibold">{data.dashboard.serial_number}</div>
      </div>
      <div>
        <div class="pt-3 text-sm tracking-wider opacity-50">Manufacture Date</div>
        <div class="pt-1 text-lg font-semibold">{data.dashboard.manufacture_date}</div>
      </div>
      <div>
        <div class="pt-3 text-sm tracking-wider opacity-50">System Version</div>
        <div class="pt-1 text-lg font-semibold">{data.dashboard.system_version}</div>
      </div>
      <div>
        <div class="pt-3 text-sm tracking-wider opacity-50">Plugin Version</div>
        <div class="whitespace-pre-line pt-1 text-lg font-semibold">
          {data.dashboard.plugin_version}
        </div>
      </div>
      {#each data.dashboard.hw_info || [] as info}
        <div>
          <div class="pt-3 text-sm tracking-wider opacity-50">{info[0]}</div>
          <div class="pt-1 text-lg font-semibold">{info[1]}</div>
        </div>
      {/each}
    </section>
  </div>
  <TaskCard dashboard={data.dashboard} cardClass="lg:col-span-2" />
  <PMCard dashboard={data.dashboard} />
  <ActivityCard dashboard={data.dashboard} />
  <BatteryCard dashboard={data.dashboard} />
  <MileageCard dashboard={data.dashboard} agv={data.agv} />
</section>
<div class="p-5 pt-20">
  <hr />
  <!-- eslint-disable-next-line svelte/no-at-html-tags -->
  <p class="pt-5">NavWiz ConfigPanel. {@html $whiteLabel.copyright_label || ''}</p>
</div>
