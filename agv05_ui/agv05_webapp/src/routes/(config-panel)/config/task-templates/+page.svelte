<script>
  import { Paginator, getModalStore, getToastStore } from '@skeletonlabs/skeleton';
  import { goto, invalidateAll, afterNavigate } from '$app/navigation';
  import * as _ from 'lodash-es';

  import ConfigLayout from 'components/ConfigLayout.svelte';
  import inView from 'actions/in-view.js';
  import taskTemplates from '$lib/shared/services/config/task-templates';
  import Operations from './components/Operations.svelte';
  import Filter from './components/Filter.svelte';
  import Perm from 'components/Perm.svelte';

  export let data;
  const modalStore = getModalStore();
  const toastStore = getToastStore();

  let addVisible = false;
  let cbAll;
  let selected = [];

  let isFilter = false;
  $: isFilter = Object.entries(data.filter).some(([_, v]) => v !== null);

  afterNavigate(() => {
    selected = [];
  });

  function onPageChange(e) {
    const page = e.detail + 1;
    goto(taskTemplates.listUrl(page));
  }

  function onDeleteCB(tt) {
    return function () {
      const modal = {
        type: 'confirm',
        title: 'Delete Task Template',
        body: `Are you sure you want to delete "${tt.name}"?`,
        response: (r) => {
          if (!r) {
            return;
          }
          deleteTaskTemplate(tt);
        }
      };
      modalStore.trigger(modal);
    };
  }

  function deleteTaskTemplate(tt) {
    taskTemplates
      .delete(tt.id)
      .then(() => {
        toastStore.trigger({
          message: taskTemplates.successDeleteMsg(tt.name),
          timeout: 3000,
          hoverable: true
        });
        invalidateAll();
      })
      .catch((e) => {
        console.log(e);
        // TODO: handle provide error message.
      });
  }

  function onCheckboxAllChanged() {
    if (cbAll.checked) {
      selected = data.taskTemplates.map((tt) => tt.id);
    } else {
      selected = [];
    }
  }

  function toggleCheckbox(e) {
    if (e.target.tagName === 'INPUT') {
      return;
    }
    var checkbox = this.querySelector('input[type="checkbox"]');
    checkbox.checked = !checkbox.checked;
    checkbox.dispatchEvent(new Event('change'));
  }

  function onFilter(e) {
    data.filter[e.detail.name] = e.detail.val;
    gotoNewFilter();
  }

  function onFilterRemove(name) {
    data.filter[name] = null;
    gotoNewFilter();
  }

  function onFilterRemoveAll() {
    for (const name in data.filter) {
      data.filter[name] = null;
    }
    gotoNewFilter();
  }

  function gotoNewFilter() {
    goto(
      taskTemplates.listUrl(
        1,
        data.filter.category,
        data.filter.is_active,
        data.filter.is_top_level
      )
    ).then(() => invalidateAll());
  }
</script>

<ConfigLayout title="Task Templates">
  <div class="flex justify-between pt-3">
    <Filter filterOptions={data.filterOptions} on:filterClick={onFilter} />
    <div class="flex flex-1 items-center space-x-2 px-2">
      {#each Object.entries(data.filter) as [name, val]}
        {#if val !== null}
          <button
            type="button"
            on:click={onFilterRemove(name)}
            class="variant-filled-surface btn btn-sm">
            {_.startCase(name)}: {val} <i class="fa-solid fa-close pl-3"></i>
          </button>
        {/if}
      {/each}
      {#if Object.entries(data.filter).some(([_, val]) => val !== null)}
        <button
          type="button"
          on:click={onFilterRemoveAll}
          class="variant-filled-surface btn btn-sm">
          Clear All<i class="fa-solid fa-close pl-3"></i>
        </button>
      {/if}
    </div>
    <Paginator
      settings={{
        page: data.page - 1,
        limit: data.limit,
        size: data.count,
        amounts: []
      }}
      on:page={onPageChange} />
  </div>
  <div class="table-container pt-3">
    <table class="table table-hover">
      <thead>
        <tr>
          <th class="cursor-pointer text-center" on:click={toggleCheckbox}>
            <input
              class="checkbox"
              type="checkbox"
              on:change={onCheckboxAllChanged}
              bind:this={cbAll}
              checked={selected.length !== 0 &&
                selected.length == data.taskTemplates.length} />
          </th>
          <th>#</th>
          <th>Name</th>
          <th class="w-28 text-center">Active</th>
          <th class="w-28 text-center">Top-level</th>
          <th>Allowed Groups</th>
          <th>Allowed Users</th>
          <th>Operation</th>
        </tr>
      </thead>
      <tbody>
        {#each data.taskTemplates as tt, i}
          <tr class:table-row-checked={selected.includes(tt.id)}>
            <td class="w-20 cursor-pointer text-center" on:click={toggleCheckbox}>
              <input class="checkbox" type="checkbox" value={tt.id} bind:group={selected} />
            </td>
            <td>{i + 1 + data.limit * (data.page - 1)}</td>
            <td>
              {tt.name}
              {#if tt.category}
                <button
                  type="button"
                  class="variant-filled-surface chip float-right"
                  on:click={() =>
                    onFilter({ detail: { name: 'category', val: tt.category } })}>
                  {tt.category}
                </button>
              {/if}
            </td>
            <td class="text-center">
              <button
                type="button"
                on:click={() =>
                  onFilter({ detail: { name: 'is_active', val: tt.is_active } })}>
                {#if tt.is_active}
                  <i class="fa-solid fa-check-circle m-auto text-xl text-green-500"></i>
                {:else}
                  <i class="fa-solid fa-times-circle m-auto text-xl text-red-500"></i>
                {/if}
              </button>
            </td>
            <td class="text-center">
              <button
                type="button"
                on:click={() =>
                  onFilter({ detail: { name: 'is_top_level', val: tt.is_top_level } })}>
                {#if tt.is_top_level}
                  <i class="fa-solid fa-check-circle m-auto text-xl text-green-500"></i>
                {:else}
                  <i class="fa-solid fa-times-circle m-auto text-xl text-red-500"></i>
                {/if}
              </button>
            </td>
            <td>
              {tt.allowed_groups && tt.allowed_groups.length
                ? tt.allowed_groups.join(', ')
                : '-'}
            </td>
            <td>
              {tt.allowed_users && tt.allowed_users.length ? tt.allowed_users.join(', ') : '-'}
            </td>
            <td>
              <Perm perms="system.change_tasktemplate">
                <a
                  href={taskTemplates.editUrl(tt.id)}
                  class="variant-filled btn mb-1 w-7 rounded p-1"
                  tip-title="Edit">
                  <i class="fa-solid fa-pencil"></i>
                </a>
              </Perm>
              <Perm perms="system.add_tasktemplate">
                <a
                  href={taskTemplates.copyUrl(tt.id)}
                  class="variant-filled btn mb-1 w-7 rounded p-1"
                  tip-title="Clone">
                  <i class="fa-solid fa-copy"></i>
                </a>
              </Perm>
              <Perm perms={['system.change_tasktemplate', 'system.view_users']}>
                <a
                  href={taskTemplates.usersUrl(tt.id)}
                  class="variant-filled btn mb-1 w-7 rounded p-1"
                  tip-title="Users">
                  <i class="fa-solid fa-users"></i>
                </a>
              </Perm>
              <Perm perms="system.delete_tasktemplate">
                <button
                  type="button"
                  class="variant-filled btn mb-1 w-7 rounded p-1"
                  on:click={onDeleteCB(tt)}
                  tip-title="Delete">
                  <i class="fa-solid fa-times"></i>
                </button>
              </Perm>
            </td>
          </tr>
        {:else}
          <tr>
            <td colspan="8">
              {isFilter ? 'No results found with filter.' : 'No task templates yet.'}
            </td>
          </tr>
        {/each}
      </tbody>
    </table>
  </div>
  <div
    class="mt-5"
    use:inView
    on:enter={() => (addVisible = true)}
    on:exit={() => (addVisible = false)}>
    <Perm perms="system.add_tasktemplate">
      <a
        href={taskTemplates.addUrl()}
        class="variant-filled-secondary btn ml-5 p-5"
        tip-title="Add new task template">
        <i class="fa-solid fa-plus"></i>
      </a>
    </Perm>
    <Perm perms="system.change_tasktemplate">
      <a
        href={taskTemplates.globalParamUrl()}
        class="variant-filled-secondary btn ml-5 p-5"
        tip-title="Edit task template global parameters">
        <i class="fa-solid fa-book"></i>
      </a>
      <a
        href={taskTemplates.variableUrl()}
        class="variant-filled-secondary btn ml-5 p-5"
        tip-title="Edit task template variables">
        <span class="px-1 leading-none">𝑥</span>
      </a>
    </Perm>
    <Operations bind:selected />
  </div>
  <Perm somePerms={['system.add_tasktemplate', 'system.change_tasktemplate']}>
    {#if !addVisible}
      <div class="variant-glass-surface fixed bottom-0 left-0 w-full">
        <Perm perms="system.add_tasktemplate">
          <a
            href={taskTemplates.addUrl()}
            class="variant-filled-secondary btn my-3 ml-5 p-5"
            tip-title="Add new task template">
            <i class="fa-solid fa-plus"></i>
          </a>
        </Perm>
        <Perm perms="system.change_tasktemplate">
          <a
            href={taskTemplates.globalParamUrl()}
            class="variant-filled-secondary btn my-3 ml-5 p-5"
            tip-title="Edit task template global parameters">
            <i class="fa-solid fa-book"></i>
          </a>
          <a
            href={taskTemplates.variableUrl()}
            class="variant-filled-secondary btn my-3 ml-5 p-5"
            tip-title="Edit task template variables">
            <span class="px-1 leading-none">𝑥</span>
          </a>
        </Perm>
        <Operations bind:selected />
      </div>
    {/if}
  </Perm>
</ConfigLayout>
