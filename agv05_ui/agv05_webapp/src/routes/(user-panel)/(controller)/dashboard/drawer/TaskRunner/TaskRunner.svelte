<script>
  import * as _ from 'lodash-es';
  import { getModalStore, getToastStore } from '@skeletonlabs/skeleton';
  import { onMount, getContext } from 'svelte';
  import { ProgressRadial } from '@skeletonlabs/skeleton';
  import { differenceInMilliseconds } from 'date-fns';
  import { writable, readonly } from 'svelte/store';

  import { alertModal } from '$lib/modal-service.js';
  import { fmsChannel } from 'stores/sock/index.js';

  import ModalIoLiveView from './components/modals/ModalIoLiveView.svelte';
  import ModalRemoteIoLiveView from './components/modals/ModalRemoteIoLiveView.svelte';
  import ModalActionLiveView from './components/modals/ModalActionLiveView.svelte';
  import ModalSetPose from './components/modals/ModalSetPose.svelte';
  import Start from './components/Start.svelte';
  import LocationInit from './components/LocationInit.svelte';
  import TaskRunner from './components/TaskRunner.svelte';
  import TransactionRunner from './components/TransactionRunner.svelte';

  const modalStore = getModalStore();
  const toastStore = getToastStore();
  const mainController = getContext('mainController');
  const user = getContext('user');
  const moduleManager = mainController.moduleService.getManager();
  const taskRunner = mainController.moduleService.getSub('task-runner');

  const activeModuleVar = writable('-');
  const activeModule = readonly(activeModuleVar);

  let batteryLow = false;
  let currentAction = '-';
  let customInitOptions = [];
  let defaultInit = null;
  let defaultModule = null;
  let executingTransaction = false;
  let fmsBroken = false;
  let fmsStatus = null;
  let initialized = false;
  let initStatus = null;
  let paused = null;
  let preInitOptions = [];
  let ready = false;
  let resuming = false;
  let started = false;
  let stationInit = [];
  let tasks = [];
  let taskTemplates = [];
  let transactions = [];
  let oriTaskTemplates = null;
  let oriTaskTemplatesAssoc = {};
  let registers = [];
  let stations = [];
  let variables = [];
  let canAddTask = false;
  let abortingToast;
  let abortingToastTime;

  $: isFmsMode = !!fmsStatus;

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

  $: canAddTask = updateCanAddTask(isFmsMode, fmsBroken, fmsStatus, $user);

  $: populateTaskTemplates($user, oriTaskTemplates, fmsBroken, fmsStatus, canAddTask);

  function populateTaskTemplates() {
    if (!oriTaskTemplates) {
      return;
    }
    taskTemplates = [];
    taskTemplates.open = [];
    taskTemplates.locked = [];
    taskTemplates.prohibited = [];
    let _canAddTask = canAddTask;
    if (!_canAddTask) {
      return;
    }
    let canAddTaskWithoutPin = _canAddTask === 1;
    let useTT = $user.has_perms('system.use_tasktemplate');
    let useTTWithPin = $user.has_perms_with_pin('system.use_tasktemplate');
    for (let tt of oriTaskTemplates) {
      oriTaskTemplatesAssoc[tt.id] = tt;
      let prohibited = isFmsMode && tt.allowed_users !== '*';
      let locked;
      if (
        useTT ||
        tt.allowed_users === '*' ||
        tt.allowed_users.indexOf($user.username) >= 0 ||
        tt.allowed_groups.indexOf($user.group) >= 0
      ) {
        locked = !canAddTaskWithoutPin;
      } else if (useTTWithPin || tt.allowed_users.indexOf($user.username_with_pin) >= 0) {
        locked = true;
      } else {
        continue;
      }
      taskTemplates.push(tt.name);
      if (prohibited) {
        taskTemplates.prohibited.push(tt);
      } else if (locked) {
        taskTemplates.locked.push(tt);
      } else {
        taskTemplates.open.push(tt);
      }
    }
  }

  function taskRunnerStarted() {
    batteryLow = false;
    customInitOptions = [];
    defaultInit = null;
    executingTransaction = false;
    fmsBroken = false;
    fmsStatus = null;
    initialized = false;
    paused = null;
    preInitOptions = [];
    resuming = false;
    stationInit = [];
    stations = null;
    taskTemplates = null;
    taskRunner.publish({
      command: 'init_status'
    });
    taskRunner.publish({
      command: 'list_templates'
    });
    taskRunner.publish({
      command: 'list_tasks'
    });
    taskRunner.publish({
      command: 'list_transactions'
    });
    taskRunner.publish({
      command: 'get_action'
    });
    taskRunner.publish({
      command: 'get_default_init'
    });
    taskRunner.publish({
      command: 'is_paused'
    });
  }

  function moduleManagerCallback(data) {
    if (data.command === 'active_module') {
      ready = true;
      if (data.module_id === 'task-runner') {
        started = true;
        taskRunnerStarted();
      } else {
        started = false;
      }
      activeModuleVar.set(data.module_id);
    } else if (data.command === 'start_module') {
      if (data.module_id === 'task-runner' && !data.success) {
        modalStore.trigger(
          alertModal(
            'Error',
            'Unable to start module "Task Runner". Is there another module running now?'
          )
        );
      }
    } else if (data.command === 'default_module') {
      defaultModule = data.default_module;
    }
  }

  function setPaused(paused) {
    taskRunner.publish({
      command: 'pause',
      pause: !!paused
    });
  }

  function showAbortingToast() {
    closeAbortingToast();
    abortingToast = toastStore.trigger({
      message: 'The task will stop after the current action completes.',
      timeout: 3000,
      hoverable: true
    });
    abortingToastTime = new Date();
  }

  function closeAbortingToast() {
    if (abortingToast && differenceInMilliseconds(new Date(), abortingToastTime) < 500) {
      toastStore.close(abortingToast);
      abortingToast = undefined;
      abortingToastTime = undefined;
    }
  }

  function taskRunnerCallback(data) {
    if (data.command === 'list_templates') {
      if (
        Array.isArray(data.custom_init) &&
        Array.isArray(data.pre_init) &&
        Array.isArray(data.task_templates) &&
        Array.isArray(data.stations) &&
        Array.isArray(data.registers)
      ) {
        customInitOptions = data.custom_init;
        stationInit = data.station_init;
        if (stationInit === 1) {
          stationInit = data.stations;
        }
        preInitOptions = data.pre_init;
        oriTaskTemplates = data.task_templates;
        stations = data.stations;
        registers = data.registers;
        variables = data.variables;
      }
    } else if (data.command === 'list_tasks') {
      if (Array.isArray(data.tasks)) {
        tasks = data.tasks;
      }
    } else if (data.command === 'list_transactions') {
      if (Array.isArray(data.transactions)) {
        transactions = data.transactions;
        executingTransaction = data.executing_transaction;
      }
    } else if (data.command === 'action') {
      currentAction = data.action || '-';
    } else if (data.command === 'outcome') {
      closeAbortingToast();
    } else if (data.command === 'init_status') {
      initialized = data.status === 'initialized';
      initStatus = data.status;
    } else if (data.command === 'default_init') {
      defaultInit = data.default_init;
    } else if (data.command === 'is_paused') {
      paused = data.paused;
      resuming = data.resuming;
      batteryLow = data.battery_low;
    }
  }

  function fmsCallback(data) {
    if (data.id === 'status') {
      let status = data.value.replace(/\n/g, '<br/>');
      let broken = data.broken;
      if (fmsStatus !== status || fmsBroken !== broken) {
        fmsStatus = status;
        fmsBroken = broken;
      }
    }
  }

  onMount(() => {
    moduleManager.subscribe(moduleManagerCallback);
    taskRunner.subscribe(taskRunnerCallback);
    fmsChannel.subscribe(fmsCallback);
    setTimeout(() => {
      moduleManager.publish({
        command: 'active_module'
      });
    }, 300);

    return () => {
      fmsChannel.unsubscribe(fmsCallback);
      taskRunner.unsubscribe(taskRunnerCallback);
      moduleManager.unsubscribe(moduleManagerCallback);
    };
  });

  function getDefault(param) {
    if (param.type === 'bool') {
      return false;
    } else if (param.type === 'int' || param.type === 'double') {
      if (param.min && param.min > 0) {
        return param.min;
      } else if (param.max && param.max < 0) {
        return param.max;
      } else {
        return 0;
      }
    } else if (param.type === 'str') {
      return '';
    } else if (param.type === 'Register') {
      return registers && registers.length ? registers[0] : null;
    } else if (param.type === 'Station') {
      return stations && stations.length ? stations[0] : null;
    } else if (param.type.startsWith('v')) {
      if (!variables) {
        return null;
      }
      let d = _.find(variables, (v) => v.vtype === param.type);
      return d ? d.name : null;
    }
  }

  function getTaskParams(template, title) {
    return new Promise(function (resolve) {
      let data = {
        params: {}
      };
      if (template['params'].length > 0) {
        for (let param of template['params']) {
          data.params[param.name] = param.default ? param.default : getDefault(param);
        }
        const modal = {
          type: 'component',
          component: 'modalTaskParams',
          title: title,
          meta: {
            template: template,
            variables: variables,
            stations: stations,
            registers: registers,
            data: data
          },
          position: 'items-center',
          response: (r) => {
            if (r) {
              resolve(r);
            } else {
              resolve(false);
            }
          }
        };
        modalStore.trigger(modal);
      } else {
        resolve({});
      }
    });
  }

  function showIo() {
    const modal = {
      type: 'component',
      component: {
        ref: ModalIoLiveView,
        props: {
          activeModule
        }
      },
      position: 'items-center'
    };
    modalStore.trigger(modal);
  }

  function showRemoteIo() {
    const modal = {
      type: 'component',
      component: {
        ref: ModalRemoteIoLiveView,
        props: {
          activeModule
        }
      },
      position: 'items-center'
    };
    modalStore.trigger(modal);
  }

  function showTaskTemplate() {
    const modal = {
      type: 'component',
      component: {
        ref: ModalActionLiveView,
        props: {
          activeModule
        }
      },
      position: 'items-center'
    };
    modalStore.trigger(modal);
  }

  function showSetPose() {
    const modal = {
      type: 'component',
      component: {
        ref: ModalSetPose,
        props: {
          activeModule
        }
      },
      position: 'items-center'
    };
    modalStore.trigger(modal);
  }
</script>

<div class="flex flex-col gap-2">
  {#if !ready}
    <div class="flex w-full max-w-2xl justify-center">
      <ProgressRadial width="w-16" />
    </div>
  {:else if ready && !started}
    <Start bind:fmsBroken bind:fmsStatus bind:defaultModule {moduleManager} {taskRunner} />
  {:else if ready && started && !initialized}
    <LocationInit
      bind:taskTemplates
      bind:fmsBroken
      bind:fmsStatus
      bind:defaultInit
      bind:customInitOptions
      bind:initStatus
      bind:stationInit
      bind:preInitOptions
      bind:paused
      bind:resuming
      {taskRunner}
      {getTaskParams}
      {setPaused} />
  {:else if ready && started && initialized && !executingTransaction}
    <TaskRunner
      bind:taskTemplates
      bind:isFmsMode
      bind:fmsBroken
      bind:fmsStatus
      bind:paused
      bind:batteryLow
      bind:resuming
      bind:tasks
      bind:currentAction
      {taskRunner}
      {getTaskParams}
      {showIo}
      {showRemoteIo}
      {showTaskTemplate}
      {showSetPose}
      {setPaused}
      {showAbortingToast} />
  {:else if ready && started && initialized && executingTransaction}
    <TransactionRunner
      bind:taskTemplates
      bind:isFmsMode
      bind:fmsBroken
      bind:fmsStatus
      bind:paused
      bind:batteryLow
      bind:resuming
      bind:transactions
      bind:tasks
      bind:currentAction
      {taskRunner}
      {showIo}
      {showRemoteIo}
      {showTaskTemplate}
      {showSetPose}
      {setPaused}
      {showAbortingToast} />
  {/if}
</div>
