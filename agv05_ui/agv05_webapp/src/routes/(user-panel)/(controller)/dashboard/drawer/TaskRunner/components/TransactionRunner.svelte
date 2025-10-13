<script>
  import * as _ from 'lodash-es';
  import { getContext } from 'svelte';
  import { getModalStore, getToastStore, ProgressRadial } from '@skeletonlabs/skeleton';

  import { agv05Task } from '$lib/shared/services/user-panel/agv05-task.service.js';
  import { agv05Transaction } from '$lib/shared/services/user-panel/agv05-transaction.service.js';
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
  export let transactions;
  export let tasks;
  export let currentAction;

  export let taskRunner;
  export let showIo;
  export let showRemoteIo;
  export let showTaskTemplate;
  export let showSetPose;
  export let setPaused;
  export let showAbortingToast;

  const modalStore = getModalStore();
  const toastStore = getToastStore();
  const user = getContext('user');

  let showTransactionsFromAllAgvs = false;
  let transactionResumePopup = null;

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

  function canResumeTransaction(t) {
    if (!$user.has_perms('system.resume_transaction')) {
      return false;
    }
    if (t.agv_id === true && t.status === 'paused' && t.error !== 1 && t.resume_data) {
      return true;
    }
    return false;
  }

  function resumeTransaction(t) {
    if (!canResumeTransaction(t)) {
      return;
    }

    if (transactionResumePopup) {
      modalStore.close(transactionResumePopup);
      transactionResumePopup = null;
    }

    let options;
    try {
      options = JSON.parse(t.resume_data);
      options = Object.entries(options).filter(([key, _]) => key !== 'action');
    } catch (error) {
      console.log('Fail to parse transaction resume data');
      return;
    }

    if (!options || options.length <= 0) {
      console.log('Error trying to resume transaction with no available options');
      return;
    }

    // TODO: refactor modal component, similar to downtime tracker modal.
    transactionResumePopup = {
      type: 'component',
      component: 'modalResumeTransaction',
      title: `Resume Transaction "${t.display_name}"`,
      subTitle: 'Resume action?',
      options,
      response: (res) => {
        if (!res) {
          return;
        }
        agv05Transaction
          .resume(t.id, res, t.error)
          .then(function (data) {
            if (data.result === false) {
              modalStore.trigger(
                alertModal(
                  `Failed to resume (${res}) transaction`,
                  data.message.replace(/\n/g, '<br/>')
                )
              );
              return;
            }
          })
          .catch((e) => {
            console.error(e);
            modalStore.trigger(httpErrorModal(`Failed to resume (${res}) transaction.`));
          });
      }
    };
    modalStore.trigger(transactionResumePopup);
  }

  function canCancelTransaction(t) {
    if (!$user.has_perms('system.cancel_transaction')) {
      return false;
    }
    if (t.agv_id === true && ['queue', 'waiting', 'cancelling'].indexOf(t.status) >= 0) {
      return true;
    }
    return false;
  }

  function cancelTransaction(t) {
    if (!canCancelTransaction(t)) {
      return;
    }
    agv05Transaction
      .cancel(t.id)
      .then(function (data) {
        if (data.result === false) {
          modalStore.trigger(
            alertModal('Failed to cancel transaction.', data.message.replace(/\n/g, '<br/>'))
          );
          return;
        }
      })
      .catch((e) => {
        console.error(e);
        modalStore.trigger(httpErrorModal('Failed to cancel transaction.'));
      });
  }

  function canAbortTransaction(t) {
    if (!$user.has_perms('system.abort_transaction')) {
      return false;
    }
    if (t.agv_id === true && ['transferring', 'aborting'].indexOf(t.status) >= 0) {
      return true;
    }
    return false;
  }

  function abortTransaction(t) {
    if (!canAbortTransaction(t)) {
      return;
    }
    agv05Transaction
      .abort(t.id)
      .then(function (data) {
        if (data.result === false) {
          modalStore.trigger(
            alertModal('Failed to stop transaction.', data.message.replace(/\n/g, '<br/>'))
          );
          return;
        }
      })
      .catch((e) => {
        console.error(e);
        modalStore.trigger(httpErrorModal('Failed to stop transaction.'));
      });
  }
</script>

{#if !taskTemplates}
  <ProgressRadial width="w-4" />
{/if}
<div class="card m-10 mt-3 flex min-h-[140px] flex-col p-5">
  {#each tasks as task}
    {#if (!isFmsMode || task.agv_id === true) && ['In_progress', 'Aborting'].indexOf(task.status) >= 0}
      <div class="relative pb-5">
        <b>Task:</b>
        {task.name}
        {#if task.name !== task.task_template}
          <span class="text-gray-500">({task.task_template})</span>
        {/if}
        {#if canCancelTask(task, $user)}
          <button
            type="button"
            class="variant-filled-error btn absolute right-0"
            tip-title="Cancel Task"
            on:click={() => cancelTask(task)}>
            <i class="fa-solid fa-xmark"></i>
          </button>
        {:else if canAbortTask(task, $user)}
          <button
            type="button"
            class="variant-filled-error btn absolute right-0"
            tip-title="Abort Task"
            on:click={() => abortTask(task)}>
            <i class="fa-solid fa-xmark"></i>
          </button>
        {/if}
      </div>
      <p class="text-sm text-gray-600">
        <b>Action:</b>
        {currentAction}
      </p>
      <p class="text-sm text-gray-600">
        <b>Progress:</b>
        {task.progress || '-'}
      </p>
      {#if task.params.length}
        <p class="text-sm text-gray-600">
          <b>Parameters:</b>
          {#each task.params as param}
            {#if !allBools(task.params) || task.hideAlt || param.value}
              <span>
                | <small><strong>{_.startCase(param.name)}</strong></small>
                {#if !allBools(task.params) || task.hideAlt}
                  <span class="pr-1">: {param.value}</span>
                {/if}
              </span>
            {/if}
          {/each}
          {#if allBools(task.params)}
            <button type="button" on:click={() => (task.hideAlt = !task.hideAlt)}>
              &nbsp;<i
                class="fa-solid text-xs {task.hideAlt
                  ? 'fa-square-caret-left'
                  : 'fa-square-caret-right'}"></i>
            </button>
          {/if}
        </p>
      {/if}
    {/if}
  {/each}
</div>
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
  <div class="card divide-y divide-solid divide-slate-400">
    <div class="flex justify-between rounded-t-xl bg-surface-400 px-4 py-2">
      Transaction Queue
      {#if isFmsMode}
        <div class="flex items-center">
          <label>
            <small>Show from all AGVs</small>
            <input
              class="checkbox ml-1"
              type="checkbox"
              bind:checked={showTransactionsFromAllAgvs} />
          </label>
        </div>
      {/if}
    </div>
    {#if $user.has_perms('app.show_panel_running_tasks')}
      <div class="space-y-2 divide-y-2 divide-slate-400 rounded-b-xl px-4 py-2">
        {#each transactions as t}
          {#if !isFmsMode || showTransactionsFromAllAgvs || t.agv_id === true || !t.agv_id}
            <div class="flex flex-row pt-2">
              <div class="w-full">
                <h2>
                  {t.display_name}
                  <span class="text-gray-600">({t.status})</span>
                </h2>
                <p class="text-sm text-gray-600">
                  Carrier ID:
                  <span>{t.carrier_id}</span>
                </p>
                <p class="text-sm text-gray-600">
                  Source Port:
                  <span>{t.src_port}</span>
                </p>
                <p class="text-sm text-gray-600">
                  Destination Port:
                  <span>{t.dst_port}</span>
                </p>
              </div>
              <div class="flex w-1/4 items-center justify-end space-x-2 align-middle">
                {#if canResumeTransaction(t, $user)}
                  <button
                    type="button"
                    class="variant-filled-success btn"
                    tip-title="Resume Transaction"
                    on:click={() => resumeTransaction(t)}>
                    <i class="fa-solid fa-play"></i>
                  </button>
                {/if}
                {#if canCancelTransaction(t, $user)}
                  <button
                    type="button"
                    class="variant-filled-success btn"
                    tip-title="Cancel Transaction"
                    on:click={() => cancelTransaction(t)}>
                    <i class="fa-solid fa-xmark"></i>
                  </button>
                {/if}
                {#if canAbortTransaction(t, $user)}
                  <button
                    type="button"
                    class="variant-filled-success btn"
                    tip-title="Abort Transaction"
                    on:click={() => abortTransaction(t)}>
                    <i class="fa-solid fa-xmark"></i>
                  </button>
                {/if}
              </div>
            </div>
          {/if}
        {/each}
        {#if transactions && !transactions.length}
          <div class="list-group-item">No running transactions.</div>
        {/if}
      </div>
    {:else}
      <div class="rounded-b-xl px-4 py-2">
        Your current user does not have the permission to view the transaction queue.
      </div>
    {/if}
  </div>
</div>
