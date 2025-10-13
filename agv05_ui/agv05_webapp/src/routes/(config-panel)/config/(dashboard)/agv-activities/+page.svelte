<script>
  import { Paginator } from '@skeletonlabs/skeleton';
  import { goto } from '$app/navigation';

  import Perm from 'components/Perm.svelte';
  import { timeDisplay } from 'stores/server-clock.js';
  import agvActivities from '$lib/shared/services/config/agv-activities';

  export let data;

  function onPageChange(e) {
    const page = e.detail + 1;
    goto(agvActivities.listUrl(page));
  }
</script>

<div class="py-3">
  <Paginator
    settings={{
      page: data.page - 1,
      limit: data.limit,
      size: data.count,
      amounts: [100]
    }}
    select="hidden"
    on:page={onPageChange} />
</div>

<div id="agv05-agv-activities" class="table-container">
  <table class="table table-hover">
    <thead>
      <tr>
        <th>Start Time</th>
        <th>End Time</th>
        <th>Duration</th>
        <th>Activity</th>
        <th>Activity Code</th>
        <Perm perms="system.change_agvactivity">
          <th width="30px"></th>
        </Perm>
      </tr>
    </thead>
    <tbody>
      {#if data.activities && data.activities.length > 0}
        {#each data.activities as obj, i}
          <tr>
            <td>{timeDisplay(obj.start)}</td>
            <td>{timeDisplay(obj.end)}</td>
            <td align="right">{obj.activity_duration}</td>
            <td>{obj.activity_display}</td>
            <td>{obj.activity}</td>
            <Perm perms="system.change_agvactivity">
              <td align="right">
                {#if i !== 0 || data.page !== 1}
                  <a
                    href="/config/agv-activity-update/{obj.id}/edit?next={agvActivities.listUrl(
                      data.page
                    )}"
                    class="btn-xs btn-default btn"
                    tip-title="Edit Activity"><i class="fa fa-pencil"></i></a>
                {/if}
              </td>
            </Perm>
          </tr>
        {/each}
      {:else}
        <tr><td colspan="20">No AGV activities yet.</td></tr>
      {/if}
    </tbody>
  </table>
</div>
