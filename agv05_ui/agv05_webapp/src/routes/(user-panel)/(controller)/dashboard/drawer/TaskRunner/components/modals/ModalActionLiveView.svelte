<script>
  import { onMount, getContext } from 'svelte';
  import { getModalStore } from '@skeletonlabs/skeleton';

  import { agv05Task } from '$lib/shared/services/user-panel/agv05-task.service.js';
  import taskTemplateViz from '$lib/shared/task-template/viz.js';
  import TaskRunnerModalLayout from './TaskRunnerModalLayout.svelte';

  // Props
  export let parent;
  export let activeModule;

  const modalStore = getModalStore();
  const mainController = getContext('mainController');
  let taskRunner = mainController.moduleService.getSub('task-runner');

  let _data;
  let curDepth;
  let curTaskTemplateName = '';
  let element;
  let viz;

  function vizInit(el) {
    element = el;
    viz = taskTemplateViz(element, {});

    // Obtain initial data
    agv05Task.templates().then(function (data) {
      viz.models.load(data);
      _data = data;
    });
  }

  function taskRunnerCallback(data) {
    if (data.command === 'action') {
      let action = data.action;
      let callStack = data.call_stack || [];

      viz.models.update({
        action: action,
        callStack: callStack
      });
      updateTitle();
    }
  }

  function updateTitle() {
    let depth = viz.models.getDepth();
    curDepth = depth !== 0 ? `(${depth})` : '';
    curTaskTemplateName = viz.models.rawTaskTemplateName() || 'No Action';
  }

  function goUp() {
    if (viz) {
      viz.models.goUp();
      updateTitle();
    }
  }

  function goDown() {
    if (viz) {
      viz.models.goDown();
      updateTitle();
    }
  }

  function resetDepth() {
    if (viz) {
      viz.models.resetDepth();
      updateTitle();
    }
  }

  taskRunner.subscribe(taskRunnerCallback);
  taskRunner.publish({
    command: 'get_action'
  });

  onMount(() => {
    vizInit(element);

    return () => {
      element = null;
      viz = null;
      taskRunner.unsubscribe(taskRunnerCallback);
    };
  });
</script>

{#if $modalStore[0]}
  <TaskRunnerModalLayout {activeModule}>
    <div class="panel-modal card flex h-[90vh] w-[90vw] flex-col rounded-3xl">
      <div class="variant-filled-secondary rounded-t-2xl p-4" style="position: relative;">
        <span class="text-center font-bold">Action Live View</span>
        <div class="float-right">
          <button type="button" class="btn btn-sm" on:click={parent.onClose}>
            <i class="fa-solid fa-xmark"></i>
          </button>
        </div>
      </div>
      <div class="flex grow flex-col space-y-2 overflow-auto p-4" on:ready>
        <div style="position: relative;">
          <span class="text-center text-2xl">{curDepth} {curTaskTemplateName}</span>
          <div class="float-right">
            <button type="button" class="btn-xs btn" on:click={goUp}>
              <i class="fa-solid fa-caret-up"></i></button>
            <button type="button" class="btn-xs btn" on:click={goDown}>
              <i class="fa-solid fa-caret-down"></i></button>
            <button type="button" class="btn-xs btn" on:click={resetDepth}>
              <i class="fa-solid fa-code"></i></button>
          </div>
        </div>
        <div class="grow">
          <svg
            style="width: 100%; height: 100%"
            xmlns="http://www.w3.org/2000/svg"
            bind:this={element}></svg>
        </div>
      </div>
    </div>
  </TaskRunnerModalLayout>
{/if}
