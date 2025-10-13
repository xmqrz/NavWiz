<script>
  import { getToastStore, getModalStore, Paginator } from '@skeletonlabs/skeleton';
  import { goto } from '$app/navigation';

  import Perm from 'components/Perm.svelte';
  import { timeDisplay } from 'stores/server-clock.js';
  import taskCompleted from '$lib/shared/services/config/task-completed';
  import ColumnModal from './components/ColumnModal.svelte';
  import diagnosticViewF from '../system-log/diagnostics/diagnostic-view';

  export let data;

  let visibleData = [
    'name',
    'template',
    'params',
    'status',
    'creationTime',
    'startTime',
    'scheduledStartTime',
    'endTime',
    'duration',
    'owner',
    'diagnostics'
  ];

  const toastStore = getToastStore();
  const modalStore = getModalStore();

  function onPageChange(e) {
    const page = e.detail + 1;
    goto(taskCompleted.listUrl(page));
  }

  function downloadDiagnostics(id) {
    taskCompleted.downloadDiagnostics(id).catch((e) => {
      console.error(e);
      toastStore.trigger({
        message: e.message,
        timeout: 3000,
        hoverable: true
      });
    });
  }

  function onColumnClicked() {
    const modal = {
      type: 'component',
      component: {
        ref: ColumnModal
      },
      init: visibleData,
      response: (r) => {
        if (!r) {
          return;
        }
        visibleData = r.selection;
      }
    };
    modalStore.trigger(modal);
  }

  function paramEntries(params) {
    try {
      return Object.entries(JSON.parse(params));
    } catch (_e) {
      return [];
    }
  }
</script>

<div class="flex items-center justify-between py-3">
  <button type="button" class="variant-filled btn h-8" on:click={onColumnClicked}>
    <i class="fa fa-columns"></i>
  </button>

  <Paginator
    class="space-y-0"
    settings={{
      page: data.page - 1,
      limit: data.limit,
      size: data.count,
      amounts: [100]
    }}
    select="hidden"
    on:page={onPageChange} />
</div>

<div
  id="agv05-tasks-completed"
  class="agv05-diagnostic-view table-container"
  use:diagnosticViewF>
  <table
    id="datatable-completed-task"
    class="table table-hover"
    data-columns="name,template,>params,status,>progress,creation-time,start-time,>scheduled-start-time,end-time,duration,owner,diagnostics">
    <thead>
      <tr>
        <th>#</th>
        <th
          class="datatable-column datatable-column-name"
          class:hidden={!visibleData.includes('name')}>Name</th>
        <th
          class="datatable-column datatable-column-template"
          class:hidden={!visibleData.includes('template')}>Template</th>
        <th
          class="datatable-column datatable-column-status"
          class:hidden={!visibleData.includes('status')}>Status</th>
        <th
          class="datatable-column datatable-column-creation-time"
          class:hidden={!visibleData.includes('creationTime')}>Creation Time</th>
        <th
          class="datatable-column datatable-column-start-time"
          class:hidden={!visibleData.includes('startTime')}>Start Time</th>
        <th
          class="datatable-column datatable-column-end-time"
          class:hidden={!visibleData.includes('endTime')}>End Time</th>
        <th
          class="datatable-column datatable-column-duration"
          class:hidden={!visibleData.includes('duration')}>Duration</th>
        <th
          class="datatable-column datatable-column-owner"
          class:hidden={!visibleData.includes('owner')}>Owner</th>
        <th
          class="datatable-column datatable-column-diagnostics"
          width="70px"
          class:hidden={!visibleData.includes('diagnostics')}></th>
      </tr>
    </thead>
    <tbody>
      {#each data.completedTasks as obj, i}
        {#if data.completedTasks && data.completedTasks.length > 0}
          <tr>
            <td>{i + 1}</td>
            <td
              class="datatable-column datatable-column-name"
              class:hidden={!visibleData.includes('name')}>{obj.name}</td>
            <td
              class="datatable-column datatable-column-template"
              class:hidden={!visibleData.includes('template')}>
              {obj.task_template ? obj.task_template : '-'}
              {#if obj.params && visibleData.includes('params')}
                <small class="datatable-column datatable-column-params text-muted">
                  <!--TODO: havent handle all_bools in backend -->
                  {#if obj.all_bools}
                    <strong data-toggle="collapse" aria-expanded="false" class="toggle-params">
                      <i class="fa fa-plus-square-o"></i>
                      <i class="fa fa-minus-square-o"></i>
                    </strong>
                    {#each paramEntries(obj.params) as [k, v]}
                      {#if v}
                        <br /><strong>{k}</strong>: {v}
                      {/if}
                    {/each}
                    <span class="collapse-inline collapse">
                      {#each paramEntries(obj.params) as [k, v]}
                        {#if !v}
                          <br /><strong>{k}</strong>: {v}
                        {/if}
                      {/each}
                    </span>
                  {:else}
                    {#each paramEntries(obj.params) as [k, v]}
                      <br /><strong>{k}</strong>: {v}
                    {/each}
                  {/if}
                </small>
              {/if}
            </td>
            <td
              class="datatable-column datatable-column-status"
              class:hidden={!visibleData.includes('status')}>
              {#if obj.executed && obj.progress}
                {obj.status}
                <small class="datatable-column datatable-column-progress text-muted">
                  <br /><strong>Progress</strong>: {obj.progress}
                </small>
              {:else}
                {obj.status}
              {/if}
            </td>
            <td
              class="datatable-column datatable-column-creation-time"
              class:hidden={!visibleData.includes('creationTime')}>
              {timeDisplay(obj.created)}
            </td>
            <td
              class="datatable-column datatable-column-start-time"
              class:hidden={!visibleData.includes('startTime')}>
              {#if obj.executed}
                {#if obj.run_start}
                  {timeDisplay(obj.run_start)}
                {/if}
                {#if obj.start_after && visibleData.includes('scheduledStartTime')}
                  <small
                    class="datatable-column datatable-column-scheduled-start-time text-muted">
                    <br /><strong>Sched</strong>: {timeDisplay(obj.start_after)}
                  </small>
                {/if}
              {/if}
            </td>
            <td
              class="datatable-column datatable-column-end-time"
              class:hidden={!visibleData.includes('endTime')}>
              {timeDisplay(obj.run_end)}
            </td>
            <td
              class="datatable-column datatable-column-duration"
              class:hidden={!visibleData.includes('duration')}>
              {obj.duration}
            </td>
            <td
              class="datatable-column datatable-column-owner"
              class:hidden={!visibleData.includes('owner')}>{obj.owner || '-'}</td>
            <td
              class="datatable-column datatable-column-diagnostics flex flex-row"
              align="right"
              class:hidden={!visibleData.includes('diagnostics')}>
              <Perm perms="system.view_log_files">
                <!--TODO: handle download and read diagnostics-->
                {#if obj.executed}
                  <button
                    type="button"
                    data-url={taskCompleted.diagnosticUrl(obj.id)}
                    data-method="POST"
                    tip-title="View Diagnostics"
                    class="diagnostic-start-view btn">
                    <i class="fa fa-table"></i>
                  </button>
                  <button
                    type="button"
                    tip-title="Download Diagnostics"
                    class="btn"
                    on:click={() => downloadDiagnostics(obj.id)}>
                    <i class="fa fa-download"></i>
                  </button>
                {/if}
              </Perm>
            </td>
          </tr>
        {:else}
          <tr><td colspan="20">No completed tasks yet.</td></tr>
        {/if}
      {/each}
    </tbody>
  </table>
</div>
