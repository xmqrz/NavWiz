<script>
  import { getToastStore } from '@skeletonlabs/skeleton';
  import { goto, invalidateAll } from '$app/navigation';

  import ConfigLayout from 'components/ConfigLayout.svelte';
  import taskTemplates from '$lib/shared/services/config/task-templates';
  import { onMount } from 'svelte';

  export let data;

  const toastStore = getToastStore();

  let active = data.taskTemplates.every((tt) => tt.is_active);
  let topLvl = data.taskTemplates.every((tt) => tt.is_top_level);
  let createSuspended = data.taskTemplates.every((tt) => tt.create_suspended);
  let activeDom;
  let topLvlDom;
  let suspendDom;
  let dirty = false;

  function triggerDirty() {
    this.indeterminate = false;
    dirty = true;
  }

  function onSubmit() {
    if (!dirty) {
      return;
    }

    let payload = {};
    if (!activeDom.indeterminate) {
      payload.is_active = active;
    }
    if (!topLvlDom.indeterminate) {
      payload.is_top_level = topLvl;
    }
    if (!suspendDom.indeterminate) {
      payload.create_suspended = createSuspended;
    }
    taskTemplates
      .massUpdate(
        data.taskTemplates.map((tt) => tt.id),
        payload
      )
      .then(() => {
        toastStore.trigger({
          message: taskTemplates.successMassUpdateMsg(data.taskTemplates.length),
          timeout: 3000,
          hoverable: true
        });
        dirty = false;
        // Prevent using preloaded data
        goto(taskTemplates.listUrl()).then(() => invalidateAll());
      })
      .catch((e) => {
        console.log(e);
        // TODO: handle provide error message.
      });
  }

  onMount(() => {
    activeDom.indeterminate = !active && data.taskTemplates.some((tt) => tt.is_active);
    topLvlDom.indeterminate = !topLvl && data.taskTemplates.some((tt) => tt.is_top_level);
    suspendDom.indeterminate =
      !createSuspended && data.taskTemplates.some((tt) => tt.create_suspended);
  });
</script>

<ConfigLayout
  title="Multiple task templates update"
  back={['Task Templates', taskTemplates.listUrl()]}>
  <div class="pt-3 lg:px-20">
    <h4 class="h4">Task Templates</h4>
    <div class="table-container max-h-96 pt-3">
      <table class="table table-hover">
        <thead>
          <tr>
            <th>#</th>
            <th>Name</th>
            <th class="w-28 text-center">Active</th>
            <th class="w-28 text-center">Top-level</th>
            <th class="w-52 text-center">Suspend task initially</th>
          </tr>
        </thead>
        <tbody>
          {#each data.taskTemplates as tt, i}
            <tr>
              <td>{i + 1}</td>
              <td>
                {tt.name}
              </td>
              <td class="text-center">
                {#if tt.is_active}
                  <i class="fa-solid fa-check-circle m-auto text-xl text-green-500"></i>
                {:else}
                  <i class="fa-solid fa-times-circle m-auto text-xl text-red-500"></i>
                {/if}
              </td>
              <td class="text-center">
                {#if tt.is_top_level}
                  <i class="fa-solid fa-check-circle m-auto text-xl text-green-500"></i>
                {:else}
                  <i class="fa-solid fa-times-circle m-auto text-xl text-red-500"></i>
                {/if}
              </td>
              <td class="text-center">
                {#if tt.create_suspended}
                  <i class="fa-solid fa-check-circle m-auto text-xl text-green-500"></i>
                {:else}
                  <i class="fa-solid fa-times-circle m-auto text-xl text-red-500"></i>
                {/if}
              </td>
            </tr>
          {/each}
        </tbody>
      </table>
    </div>
  </div>
  <div class="pt-4 opacity-50 lg:pl-20">
    {data.taskTemplates.length} task templates selected
  </div>
  <div class="pt-3 lg:pl-20">
    <span class="p-3 font-medium">Options</span>
    <div class="space-y-5 p-3">
      <label class="flex items-center space-x-2">
        <input
          class="checkbox"
          type="checkbox"
          bind:checked={active}
          bind:this={activeDom}
          on:change={triggerDirty} />
        <p>Active</p>
      </label>
      <label class="flex items-center space-x-2">
        <input
          class="checkbox"
          type="checkbox"
          bind:checked={topLvl}
          bind:this={topLvlDom}
          on:change={triggerDirty} />
        <p>Top-level template</p>
      </label>
      <label class="flex items-center space-x-2">
        <input
          class="checkbox"
          type="checkbox"
          disabled={!topLvl}
          bind:checked={createSuspended}
          bind:this={suspendDom}
          on:change={triggerDirty} />
        <p>Suspend task initially</p>
      </label>
      <button
        type="submit"
        class="variant-filled-primary btn"
        on:click|preventDefault={onSubmit}
        disabled={!dirty}>
        Submit
      </button>
    </div>
  </div>
</ConfigLayout>
