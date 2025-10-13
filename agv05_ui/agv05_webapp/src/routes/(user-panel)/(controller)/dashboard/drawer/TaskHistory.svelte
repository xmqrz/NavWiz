<script>
  import { map, startCase } from 'lodash-es';
  import { onMount } from 'svelte';
  import { ProgressRadial } from '@skeletonlabs/skeleton';

  import { agv05Task } from '$lib/shared/services/user-panel/agv05-task.service.js';
  import { allBools } from '$lib/utils';

  let tasks = [];

  function populateCompletedTasks() {
    tasks = [];
    agv05Task.queryCompleted(30).then(function (data) {
      for (let t of data.results) {
        try {
          t.params = map(JSON.parse(t.params), (v, k) => ({
            name: k,
            value: v
          }));
        } catch (e) {
          t.params = [];
        }
      }
      tasks = data.results;
    });
  }

  onMount(() => {
    populateCompletedTasks();
  });
</script>

<div class="card mt-3">
  <div class="rounded-t-xl bg-surface-400 px-4 py-2">
    <strong>Task History:</strong> Showing the last 30 completed tasks.
    <button
      type="button"
      class="variant-filled-tertiary btn btn-sm"
      on:click={populateCompletedTasks}>
      <i class="fa-solid fa-rotate-right mr-2"></i> Refresh
    </button>
  </div>
  {#if tasks}
    <div class="divide-y divide-solid divide-surface-400 rounded-b-xl">
      {#if tasks && !tasks.length}
        <div class="px-4 py-2">No completed tasks yet.</div>
      {:else}
        {#each tasks as task}
          <div class="px-4 py-2">
            <h2 class="font-bold">
              {task.name}
              {#if task.name !== task.task_template}
                <span class="dark">({task.task_template})</span>
              {/if}
            </h2>
            {#if task.params.length}
              {@const isAllBools = allBools(task.params)}
              <p class="text-slate-600 dark:text-slate-300">
                Parameters:
                {#each task.params as param}
                  {#if !isAllBools || task.hideAlt || param.value}
                    <span>
                      | <button type="button"><strong>{startCase(param.name)}</strong></button>
                      {#if !isAllBools || task.hideAlt}
                        <span>: {param.value}</span>
                      {/if}
                    </span>
                  {/if}
                {/each}
                {#if isAllBools}
                  {#if task.hideAlt}
                    <button type="button" on:click={() => (task.hideAlt = !task.hideAlt)}
                      >&nbsp;<i class="fa fa-square-caret-left"></i></button>
                  {:else}
                    <button type="button" on:click={() => (task.hideAlt = !task.hideAlt)}
                      >&nbsp;<i class="fa fa-square-caret-right"></i></button>
                  {/if}
                {/if}
              </p>
            {/if}
            <p class="text-slate-600 dark:text-slate-300">Status: {startCase(task.status)}</p>
            {#if ['Completed', 'Aborted'].indexOf(task.status) >= 0 && task.progress}
              <p class="text-slate-600 dark:text-slate-300">
                Progress: {task.progress}
              </p>
            {/if}
          </div>
        {/each}
      {/if}
    </div>
  {:else if !tasks}
    <div class="flex justify-center rounded-b-xl px-4 py-2">
      <ProgressRadial width="w-4" />
    </div>
  {/if}
</div>
