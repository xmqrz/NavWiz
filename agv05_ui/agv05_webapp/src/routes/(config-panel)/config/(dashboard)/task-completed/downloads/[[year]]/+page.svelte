<script>
  import { format } from 'date-fns';
  import taskCompleted from '$lib/shared/services/config/task-completed';

  export let data;

  // display month
  function dy(month) {
    return format(month, 'yyyy');
  }

  function download(monthInteger) {
    let payload = {
      year: dy(data.current),
      month: monthInteger
    };
    taskCompleted.downloadTaskCompleted(payload).catch((e) => {
      console.error(e);
    });
  }
</script>

<div class="space-y-4 py-4">
  <div class="table-container">
    <table class="table">
      <thead>
        <tr>
          <th class="!p-2 text-center">
            {#if data.previous}
              <a class="float-left" href="/config/task-completed/downloads/{data.previous}">
                <i class="fa fa-lg fa-arrow-circle-left"></i>
              </a>
            {/if}
            {dy(data.current)}
            {#if data.next}
              <a class="float-right" href="/config/task-completed/downloads/{data.next}">
                <i class="fa fa-lg fa-arrow-circle-right"></i>
              </a>
            {/if}
          </th>
        </tr>
      </thead>
      <tbody>
        <tr class="grid grid-cols-4 p-4">
          {#if data.monthlyDataSummary}
            {#each data.monthlyDataSummary as obj, i}
              {#if i % 4 === 0}
                <div class="row col-span-4"></div>
              {/if}
              <div class="text-center">
                {#if obj.exists}
                  <button
                    type="button"
                    class="cursor-pointer text-blue-500"
                    on:click={() => download(obj.month_number)}>{obj.month}</button>
                {:else}
                  {obj.month}
                {/if}
              </div>
            {/each}
          {/if}
        </tr>
      </tbody>
    </table>
  </div>
</div>
