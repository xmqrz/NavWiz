<script>
  import { getDrawerStore, getToastStore, ProgressRadial } from '@skeletonlabs/skeleton';
  import { onMount, getContext } from 'svelte';

  import Perm from 'components/Perm.svelte';
  import registerCustomElement from './custom-elements';
  import { getEnv } from 'stores/auth';
  import KeyboardSpacer from 'components/keyboard/KeyboardSpacer.svelte';
  import { statusAPIChannel } from 'stores/sock';
  import TaskRunner from './drawer/TaskRunner/TaskRunner.svelte';

  export let data;

  const toastStore = getToastStore();
  const drawerStore = getDrawerStore();

  const user = getContext('user');
  const mainController = getContext('mainController');
  const moduleManager = mainController.moduleService.getManager();
  const taskRunner = mainController.moduleService.getSub('task-runner');

  let init;
  let destroy;

  function openTaskRunner() {
    drawerStore.open({
      id: 'task-runner',
      position: 'bottom',
      height: 'h-full',
      padding: 'pt-12',
      rounded: 'rounded-t-3xl'
    });
  }

  function openTaskHistory(pos = 'bottom') {
    if (['right', 'left'].indexOf(pos) < 0) {
      drawerStore.open({
        id: 'task-history',
        position: pos,
        height: 'h-full',
        padding: 'pt-12',
        rounded: 'rounded-t-3xl'
      });
    } else {
      drawerStore.open({
        id: 'task-history',
        position: pos,
        height: 'h-full',
        width: 'w-full',
        padding: 'pt-12',
        rounded: 'rounded-t-3xl'
      });
    }
  }

  function taskRunnerCallback(data) {
    if (data.command === 'error') {
      toastStore.trigger({
        background: 'variant-filled-error',
        message: `Task "${data.task}" Error: ${data.error}`,
        timeout: 3000,
        hoverable: true
      });
    } else if (data.command === 'outcome') {
      toastStore.trigger({
        message: `Task "${data.task}" Outcome: ${data.outcome}`,
        timeout: 3000,
        hoverable: true
      });
    }
  }

  onMount(() => {
    const customElementAPI = {
      moduleManager,
      taskRunner,
      openTaskRunner,
      openTaskHistory
    };
    registerCustomElement(customElementAPI);
    taskRunner.subscribe(taskRunnerCallback);
    load();

    return () => {
      taskRunner.unsubscribe(taskRunnerCallback);
      if (destroy) {
        destroy();
      }
    };
  });

  function load() {
    if (!data.entry) {
      return;
    }
    import(/* @vite-ignore */ data.entry)
      .then((module) => {
        init = module.init;
      })
      .catch((error) => {
        data.entry = undefined;
        console.error('Error loading Dashboard plugin:', error);
      });
  }

  function dashboard(dom) {
    const dashboardAPI = {
      taskRunnerChannel: taskRunner,
      statusChannel: statusAPIChannel,
      isAgvPanel: () => $user.is_agv_panel(),
      isTrackless: () => getEnv('TRACKLESS'),
      openTaskRunner,
      openTaskHistory
    };
    const fn = init(dom, dashboardAPI);
    if (fn instanceof Function) {
      destroy = fn;
    }
  }

  function openTaskHistoryBottom() {
    openTaskHistory();
  }
</script>

<div>
  {#if data.entry}
    {#if !init}
      <!-- TODO: use placeholder loading instead?... -->
      <ProgressRadial width="w-24 p-5" />
    {:else}
      <div use:dashboard></div>
    {/if}
    <KeyboardSpacer class="bg-white" />
  {:else}
    <TaskRunner />
    <Perm perms="app.show_panel_completed_tasks">
      <div class="float-right-btn">
        <button
          tip-title="Task History"
          type="button"
          on:click={openTaskHistoryBottom}
          class="variant-filled-surface btn m-5 bg-surface-800 p-4 py-6">
          <i class="fa-solid fa-history fa-lg text-gray-200"></i>
        </button>
      </div>
    </Perm>
  {/if}
</div>

<style lang="postcss">
  .float-right-btn {
    @apply fixed;
    @apply bottom-0 right-0;
    @apply flex flex-col space-y-3;
  }
</style>
