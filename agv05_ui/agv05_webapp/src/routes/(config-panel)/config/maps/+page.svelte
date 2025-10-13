<script>
  import { Paginator, getModalStore, getToastStore } from '@skeletonlabs/skeleton';

  import ConfigLayout from 'components/ConfigLayout.svelte';
  import { goto, invalidateAll } from '$app/navigation';
  import { getEnv } from 'stores/auth';
  import inView from 'actions/in-view.js';
  import maps from '$lib/shared/services/config/maps';
  import { timeDisplay } from 'stores/server-clock.js';
  import Perm from 'components/Perm.svelte';

  export let data;
  const modalStore = getModalStore();
  const toastStore = getToastStore();

  const TRACKLESS = getEnv('TRACKLESS');
  let addVisible = false;

  function onPageChange(e) {
    const page = e.detail + 1;
    goto(maps.listUrl(page));
  }

  function onDeleteCB(map) {
    return function () {
      const modal = {
        type: 'confirm',
        title: 'Delete Map',
        body: `Are you sure you want to delete "${map.name}"?`,
        response: (r) => {
          if (!r) {
            return;
          }
          removeMap(map);
        }
      };
      modalStore.trigger(modal);
    };
  }

  function removeMap(map) {
    maps
      .remove(map.id)
      .then(() => {
        toastStore.trigger({
          message: maps.successDeleteMsg(map.name),
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

  let actionBtn = [
    {
      title: 'Edit Active Map',
      icon: 'fa-pencil',
      perm: 'system.change_map',
      url: maps.activeUrl()
    },
    {
      title: 'Edit Map Teleportation',
      icon: 'fa-code-compare fa-rotate-90',
      perm: 'system.change_map',
      url: maps.teleportUrl()
    },
    {
      title: 'Edit Map Transition Trigger',
      icon: 'fa-arrow-right-arrow-left',
      perm: 'system.change_map',
      url: maps.tTriggerUrl()
    },
    {
      title: 'Edit Map Parameters',
      icon: 'fa-book',
      perm: 'system.change_map',
      url: maps.paramUrl()
    }
  ];

  if (!TRACKLESS) {
    actionBtn = [
      {
        title: 'Add New Map',
        icon: 'fa-plus',
        perm: 'system.add_map',
        url: maps.addUrl()
      },
      ...actionBtn
    ];
  }

  let allPerms = actionBtn.reduce((acc, current) => {
    acc.add(current.perm);
    return acc;
  }, new Set());
  allPerms = [...allPerms];
</script>

<ConfigLayout title="Maps">
  <div class="flex justify-end pt-3">
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
          <th>#</th>
          <th>Name</th>
          {#if TRACKLESS}
            <th>Mapping Session Start Time</th>
          {/if}
          <th>Operation</th>
        </tr>
      </thead>
      <tbody>
        {#each data.maps as map, i}
          <tr>
            <td>{i + 1 + data.limit * (data.page - 1)}</td>
            <td>
              {map.name}
              {#if map.active}
                <span class="variant-filled-success chip float-right">Active</span>
              {/if}
            </td>
            {#if TRACKLESS}
              <td>{timeDisplay(map.created)}</td>
            {/if}
            <td>
              {#if TRACKLESS}
                <Perm perms="systemx.change_mapocg">
                  <a
                    href={maps.ocgsUrl(map.id)}
                    class="variant-filled btn mb-1 w-7 rounded p-1"
                    tip-title="View Raw Maps">
                    <i class="fa-solid fa-image"></i>
                  </a>
                </Perm>
              {/if}
              <Perm perms="system.change_map">
                <a
                  href={maps.layoutUrl(map.id)}
                  class="variant-filled btn mb-1 w-7 rounded p-1"
                  tip-title="Edit Map Layout">
                  <i class="fa-solid fa-map"></i>
                </a>
                <a
                  href={maps.annotationUrl(map.id)}
                  class="variant-filled btn mb-1 w-7 rounded p-1"
                  tip-title="Edit Map Annotation">
                  <i class="fa-solid fa-commenting"></i>
                </a>
              </Perm>
              <Perm perms="system.change_map">
                <a
                  href={maps.editUrl(map.id)}
                  class="variant-filled btn mb-1 w-7 rounded p-1"
                  tip-title="Edit Properties">
                  <i class="fa-solid fa-pencil"></i>
                </a>
              </Perm>
              <Perm perms="system.add_map">
                <a
                  href={maps.copyUrl(map.id)}
                  class="variant-filled btn mb-1 w-7 rounded p-1"
                  tip-title="Clone">
                  <i class="fa-solid fa-copy"></i>
                </a>
              </Perm>
              <Perm perms="system.delete_map">
                <button
                  type="button"
                  class="variant-filled btn mb-1 w-7 rounded p-1"
                  on:click={onDeleteCB(map)}
                  tip-title="Delete">
                  <i class="fa-solid fa-times"></i>
                </button>
              </Perm>
            </td>
          </tr>
        {:else}
          <tr>
            <td colspan="3">No maps yet.</td>
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
    {#each actionBtn as btn}
      <Perm perms={btn.perm}>
        <a
          href={btn.url}
          class="variant-filled-secondary btn mr-5 p-5 text-white"
          tip-title={btn.title}>
          <i class="fa-solid {btn.icon}"></i>
        </a>
      </Perm>
    {/each}
  </div>
  <Perm somePerms={allPerms}>
    {#if !addVisible}
      <div class="variant-glass-surface fixed bottom-0 left-0 w-full">
        {#each actionBtn as btn}
          <Perm perms={btn.perm}>
            <a
              href={btn.url}
              class="variant-filled-secondary btn my-3 ml-5 p-5 text-white"
              tip-title={btn.title}>
              <i class="fa-solid {btn.icon}"></i>
            </a>
          </Perm>
        {/each}
      </div>
    {/if}
  </Perm>
  <div class="mt-5">
    <nav class="list-nav">
      <div class="font-semibold">Active Map:</div>
      <ul class="md:max-w-lg">
        {#each data.active as map}
          <li>
            <a href={maps.layoutUrl(map.id)}>
              <span class="badge bg-gray-500 text-white">
                <i class="fa-solid fa-arrow-right"></i>
              </span>
              <span class="flex-auto">{map.name}</span>
            </a>
          </li>
        {/each}
      </ul>
    </nav>
  </div>
</ConfigLayout>
