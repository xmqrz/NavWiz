<script>
  import * as _ from 'lodash-es';
  import { getContext } from 'svelte';
  import { getModalStore, getToastStore, ProgressRadial } from '@skeletonlabs/skeleton';

  import { agv05Task } from '$lib/shared/services/user-panel/agv05-task.service.js';
  import { alertModal, httpErrorModal } from '$lib/modal-service.js';
  import { allBools } from '$lib/utils';
  import RunnerController from './RunnerController.svelte';

  export let taskTemplates;
  export let isFmsMode;
  export let fmsBroken;
  export let fmsStatus;
  export let paused;
  export let batteryLow;
  export let resuming;
  export let tasks;
  export let currentAction;

  export let taskRunner;
  export let getTaskParams;
  export let showIo;
  export let showRemoteIo;
  export let showTaskTemplate;
  export let showSetPose;
  export let setPaused;
  export let showAbortingToast;

  const modalStore = getModalStore();
  const toastStore = getToastStore();
  const user = getContext('user');

  let canAddTask = false;
  let showTpl = 0;
  let showTasksFromAllAgvs = false;

  $: canAddTask = updateCanAddTask(isFmsMode, fmsBroken, fmsStatus, $user);

  function updateCanAddTask(isFmsMode, fmsBroken, fmsStatus, $user) {
    if (isFmsMode) {
      if (fmsBroken || fmsStatus !== 'Ready' || !$user.has_perms('agvccs.add_task')) {
        return false;
      }
    }
    if ($user.has_perms('system.add_task')) {
      return 1;
    }
    if ($user.has_perms_with_pin('system.add_task')) {
      return -1;
    }
    return false;
  }

  function addTask(template) {
    getTaskParams(template, `Add task "${template.name}"`).then(function (res) {
      if (res) {
        agv05Task
          .post({
            task_template_id: template.id,
            params: res
          })
          .then(function (data) {
            if (data.result === false) {
              modalStore.trigger(
                alertModal('Failed to add task.', data.message.replace(/\n/g, '<br/>'))
              );
            }
          })
          .catch((e) => {
            console.error(e);
            modalStore.trigger(httpErrorModal('Failed to add task.'));
          });
      }
    });
  }

  function canPrioritizeTask(task) {
    if (task.status !== 'Pending') {
      return false;
    }
    if (isFmsMode) {
      if (
        !(
          $user.has_perms('agvccs.prioritize_task') ||
          ($user.has_perms('agvccs.prioritize_own_task') && task.agv_id === true)
        )
      ) {
        return false;
      }
    }
    if (
      $user.has_perms('system.prioritize_task') ||
      $user.has_perms_with_pin('system.prioritize_task')
    ) {
      return true;
    }
    if ($user.has_perms('system.prioritize_own_task') && task.owner === $user.username) {
      return true;
    }
    if (
      $user.has_perms_with_pin('system.prioritize_own_task') &&
      (task.owner === $user.username || task.owner === $user.username_with_pin)
    ) {
      return true;
    }
    return false;
  }

  function prioritizeTask(task) {
    if (!canPrioritizeTask(task)) {
      return;
    }
    agv05Task
      .prioritize(task.id, task.fms_task_id)
      .then(function (data) {
        if (data.result === false) {
          modalStore.trigger(
            alertModal('Failed to prioritize task.', data.message.replace(/\n/g, '<br/>'))
          );
        }
      })
      .catch((e) => {
        console.error(e);
        modalStore.trigger(httpErrorModal('Failed to prioritize task.'));
      });
  }

  function canSuspendTask(task) {
    // Todo: More control over which task can be suspended.
    // Disable `suspendTask` at the moment.
    if (task) {
      return false;
    }

    if (task.status !== 'Pending') {
      return false;
    }
    if (isFmsMode) {
      if (
        !(
          $user.has_perms('agvccs.suspend_task') ||
          ($user.has_perms('agvccs.suspend_own_task') && task.agv_id === true)
        )
      ) {
        return false;
      }
    }
    if (
      $user.has_perms('system.suspend_task') ||
      $user.has_perms_with_pin('system.suspend_task')
    ) {
      return true;
    }
    if ($user.has_perms('system.suspend_own_task') && task.owner === $user.username) {
      return true;
    }
    if (
      $user.has_perms_with_pin('system.suspend_own_task') &&
      (task.owner === $user.username || task.owner === $user.username_with_pin)
    ) {
      return true;
    }
    return false;
  }

  function suspendTask(task) {
    if (!canSuspendTask(task)) {
      return;
    }
    agv05Task
      .suspend(task.id, task.fms_task_id)
      .then(function (data) {
        if (data.result === false) {
          modalStore.trigger(
            alertModal('Failed to suspend task.', data.message.replace(/\n/g, '<br/>'))
          );
        }
      })
      .catch((e) => {
        console.error(e);
        modalStore.trigger(httpErrorModal('Failed to suspend task.'));
      });
  }

  function canResumeTask(task) {
    if (task.status !== 'Suspended') {
      return false;
    }
    if (isFmsMode) {
      if (
        !(
          $user.has_perms('agvccs.resume_task') ||
          ($user.has_perms('agvccs.resume_own_task') && task.agv_id === true)
        )
      ) {
        return false;
      }
    }
    if (
      $user.has_perms('system.resume_task') ||
      $user.has_perms_with_pin('system.resume_task')
    ) {
      return true;
    }
    if ($user.has_perms('system.resume_own_task') && task.owner === $user.username) {
      return true;
    }
    if (
      $user.has_perms_with_pin('system.resume_own_task') &&
      (task.owner === $user.username || task.owner === $user.username_with_pin)
    ) {
      return true;
    }
    return false;
  }

  function resumeTask(task) {
    if (!canResumeTask(task)) {
      return;
    }
    agv05Task
      .resume(task.id, task.fms_task_id)
      .then(function (data) {
        if (data.result === false) {
          modalStore.trigger(
            alertModal('Failed to resume task', data.message.replace(/\n/g, '<br/>'))
          );
        }
      })
      .catch((e) => {
        console.error(e);
        modalStore.trigger(httpErrorModal('Failed to resume task.'));
      });
  }

  function canCancelTask(task) {
    if (['Suspended', 'Pending', 'Cancelling'].indexOf(task.status) < 0) {
      return false;
    }
    if (isFmsMode) {
      if (
        !(
          $user.has_perms('agvccs.cancel_task') ||
          ($user.has_perms('agvccs.cancel_own_task') && task.agv_id === true)
        )
      ) {
        return false;
      }
    }
    if (
      $user.has_perms('system.cancel_task') ||
      $user.has_perms_with_pin('system.cancel_task')
    ) {
      return true;
    }
    if ($user.has_perms('system.cancel_own_task') && task.owner === $user.username) {
      return true;
    }
    if (
      $user.has_perms_with_pin('system.cancel_own_task') &&
      (task.owner === $user.username || task.owner === $user.username_with_pin)
    ) {
      return true;
    }
    return false;
  }

  function cancelTask(task) {
    if (!canCancelTask(task)) {
      return;
    }
    agv05Task
      .cancel(task.id, task.fms_task_id)
      .then(function (data) {
        if (data.result === false) {
          modalStore.trigger(
            alertModal('Failed to cancel task.'),
            data.message.replace(/\n/g, '<br/>')
          );
          return;
        }
        toastStore.trigger({
          message: `Task "${task.name}" Outcome: Cancelled`,
          timeout: 3000,
          hoverable: true
        });
      })
      .catch((e) => {
        console.error(e);
        modalStore.trigger(httpErrorModal('Failed to cancel task.'));
      });
  }

  function canAbortTask(task) {
    if (['In_progress', 'Aborting'].indexOf(task.status) < 0) {
      return false;
    }
    if (isFmsMode) {
      if (
        !(
          $user.has_perms('agvccs.abort_task') ||
          ($user.has_perms('agvccs.abort_own_task') && task.agv_id === true)
        )
      ) {
        return false;
      }
    }
    if (
      $user.has_perms('system.abort_task') ||
      $user.has_perms_with_pin('system.abort_task')
    ) {
      return true;
    }
    if ($user.has_perms('system.abort_own_task') && task.owner === $user.username) {
      return true;
    }
    if (
      $user.has_perms_with_pin('system.abort_own_task') &&
      (task.owner === $user.username || task.owner === $user.username_with_pin)
    ) {
      return true;
    }
    return false;
  }

  function abortTask(task) {
    if (!canAbortTask(task)) {
      return;
    }
    agv05Task
      .abort(task.id, task.fms_task_id)
      .then(function (data) {
        if (data.result === false) {
          modalStore.trigger(
            alertModal('Failed to abort task.'),
            data.message.replace(/\n/g, '<br/>')
          );
          return;
        }
        showAbortingToast();
      })
      .catch((e) => {
        console.error(e);
        modalStore.trigger(httpErrorModal('Failed to stop task.'));
      });
  }
</script>

{#if !taskTemplates}
  <ProgressRadial width="w-4" />
{:else if $user.has_perms('app.show_panel_call_buttons') && canAddTask}
  <div class="pb-3">
    {#if taskTemplates.length === 0}
      <div>No task template defined.</div>
    {/if}
    {#each taskTemplates.open as _, tplIndex}
      {#if tplIndex % 3 === 0}
        <div class="mt-3 grid grid-cols-1 gap-3 sm:grid-cols-2 lg:grid-cols-3">
          {#each [0, 1, 2] as i}
            {@const tpl = taskTemplates.open[tplIndex + i]}
            {#if tpl}
              <button
                type="button"
                class="variant-filled-surface btn btn-lg w-full bg-surface-300"
                on:click={() => addTask(tpl)}>
                <span class="truncate">
                  {tpl.name}
                </span>
              </button>
            {/if}
          {/each}
        </div>
      {/if}
    {/each}
    {#if taskTemplates.locked.length || taskTemplates.prohibited.length}
      <div>
        {#if taskTemplates.locked.length}
          <button
            type="button"
            class="btn btn-icon"
            on:click={() => (showTpl = showTpl === 1 ? 0 : 1)}>
            <i class="fa-solid fa-lock fa-2x"></i>
          </button>
        {/if}
        {#if taskTemplates.prohibited.length}
          <button
            type="button"
            class="btn btn-icon"
            on:click={() => (showTpl = showTpl === 2 ? 0 : 2)}>
            <i class="fa-solid fa-circle-minus fa-2x"></i>
          </button>
        {/if}
      </div>
    {/if}
    {#if showTpl === 1 && taskTemplates.locked.length}
      <div
        class="grid grid-cols-1 gap-2 rounded-2xl bg-yellow-500 p-5 sm:grid-cols-2 lg:grid-cols-3">
        {#each taskTemplates.locked as tpl}
          <button
            type="button"
            class="variant-filled-surface btn btn-lg w-full bg-surface-300"
            on:click={() => addTask(tpl)}>
            <i class="fa-solid fa-lock"></i>
            <span class="truncate">
              {tpl.name}
            </span>
          </button>
        {/each}
      </div>
    {/if}
    {#if showTpl === 2 && taskTemplates.prohibited.length}
      <div
        class="grid grid-cols-1 gap-2 rounded-2xl bg-red-500 p-5 sm:grid-cols-2 lg:grid-cols-3">
        {#each taskTemplates.prohibited as tpl}
          <button
            type="button"
            class="variant-filled-surface btn btn-lg w-full bg-surface-300"
            on:click={() => addTask(tpl)}>
            <i class="fa-solid fa-circle-minus"></i>
            <span class="truncate">
              {tpl.name}
            </span>
          </button>
        {/each}
      </div>
    {/if}
  </div>
{/if}
<RunnerController
  bind:isFmsMode
  bind:fmsBroken
  bind:fmsStatus
  bind:paused
  bind:batteryLow
  bind:resuming
  {taskRunner}
  {showIo}
  {showRemoteIo}
  {showTaskTemplate}
  {showSetPose}
  {setPaused} />
<div>
  <div class="card">
    <div class="flex justify-between rounded-t-xl bg-surface-400 px-4 py-2">
      Task Queue
      {#if isFmsMode}
        <div class="flex items-center">
          <label>
            <small>Show from all AGVs</small>
            <input class="checkbox ml-1" type="checkbox" bind:checked={showTasksFromAllAgvs} />
          </label>
        </div>
      {/if}
    </div>
    {#if $user.has_perms('app.show_panel_running_tasks')}
      <div class="space-y-2 divide-y divide-surface-400 rounded-b-xl px-4 py-2">
        {#each tasks as task}
          {#if !isFmsMode || showTasksFromAllAgvs || task.agv_id === true || (!task.agv_id && taskTemplates && taskTemplates.includes(task.task_template))}
            <div class="flex flex-row pt-2">
              <div class="w-full">
                <h2>
                  {task.name}
                  {#if task.name !== task.task_template}
                    <span class="text-gray-500">({task.task_template})</span>
                  {/if}
                </h2>
                {#if task.params.length}
                  {@const isAllBools = allBools(task.params)}
                  <p class="text-sm text-gray-600">
                    Parameters:
                    {#each task.params as param}
                      {#if !isAllBools || task.hideAlt || param.value}
                        <span>
                          | <small><strong>{_.startCase(param.name)}</strong></small>
                          {#if !isAllBools || task.hideAlt}
                            <span class="pr-1">: {param.value}</span>
                          {/if}
                        </span>
                      {/if}
                    {/each}
                    {#if isAllBools}
                      <button type="button" on:click={() => (task.hideAlt = !task.hideAlt)}>
                        &nbsp;<i
                          class="fa-solid text-xs {task.hideAlt
                            ? 'fa-square-caret-left'
                            : 'fa-square-caret-right'}"></i>
                      </button>
                    {/if}
                  </p>
                {/if}
                {#if task.agv}
                  <p class="text-sm text-gray-600">
                    Agv:
                    <span
                      class="rounded px-1 pb-0.5"
                      class:variant-filled-secondary={task.agv_id === true}>
                      {task.agv}
                    </span>
                  </p>
                {/if}
                <p class="text-sm text-gray-600">
                  Status: <span class:variant-filled-warning={task.status === 'Suspended'}>
                    {_.startCase(task.status)}</span>
                </p>
                {#if (!isFmsMode || task.agv_id === true) && ['In_progress', 'Aborting'].indexOf(task.status) >= 0}
                  <p class="text-sm text-gray-600">
                    Action: {currentAction}
                  </p>
                {/if}
                {#if ['In_progress', 'Aborting'].indexOf(task.status) >= 0 && task.progress}
                  <p class="text-sm text-gray-600">
                    Progress: {task.progress}
                  </p>
                {/if}
              </div>
              <div class="flex w-1/4 items-center justify-end space-x-2 align-middle">
                {#if canPrioritizeTask(task, $user)}
                  <button
                    type="button"
                    class="variant-filled-success btn"
                    tip-title="Prioritize Task"
                    on:click={() => prioritizeTask(task)}>
                    <i class="fa-solid fa-arrow-up"></i>
                  </button>
                {/if}
                {#if canSuspendTask(task, $user)}
                  <button
                    type="button"
                    class="variant-filled-warning btn"
                    tip-title="Suspend Task"
                    on:click={() => suspendTask(task)}>
                    <i class="fa-solid fa-pause"></i>
                  </button>
                {/if}
                {#if canResumeTask(task, $user)}
                  <button
                    type="button"
                    class="variant-filled-success btn"
                    tip-title="Resume Task"
                    on:click={() => resumeTask(task)}>
                    <i class="fa-solid fa-play"></i>
                  </button>
                {/if}
                {#if canCancelTask(task, $user)}
                  <button
                    type="button"
                    class="variant-filled-error btn"
                    tip-title="Cancel Task"
                    on:click={() => cancelTask(task)}>
                    <i class="fa-solid fa-xmark"></i>
                  </button>
                {/if}
                {#if canAbortTask(task, $user)}
                  <button
                    type="button"
                    class="variant-filled-error btn"
                    tip-title="Abort Task"
                    on:click={() => abortTask(task)}>
                    <i class="fa-solid fa-xmark"></i>
                  </button>
                {/if}
              </div>
            </div>
          {/if}
        {/each}
        {#if tasks && !tasks.length}
          <div class="list-group-item">No running tasks.</div>
        {/if}
      </div>
    {:else}
      <div class="rounded-b-xl px-4 py-2">
        Your current user does not have the permission to view the task queue.
      </div>
    {/if}
  </div>
</div>
