<script>
  import { Paginator, getModalStore, getToastStore } from '@skeletonlabs/skeleton';

  import ConfigLayout from 'components/ConfigLayout.svelte';
  import MapEditorTab from '../../components/MapEditorTab.svelte';
  import { goto, invalidateAll } from '$app/navigation';
  import { timeDisplay } from 'stores/server-clock.js';
  import maps from '$lib/shared/services/config/maps';
  import Perm from 'components/Perm.svelte';

  export let data;
  const modalStore = getModalStore();
  const toastStore = getToastStore();

  function onPageChange(e) {
    const page = e.detail + 1;
    goto(maps.ocgsUrl(data.mapID, page));
  }

  function onDeleteCB(ocg) {
    return function () {
      const modal = {
        type: 'confirm',
        title: 'Delete Raw Map',
        body: `Are you sure you want to delete "${ocg.display_name}"?`,
        response: (r) => {
          if (!r) {
            return;
          }
          removeOcg(ocg);
        }
      };
      modalStore.trigger(modal);
    };
  }

  function removeOcg(_ocg) {
    maps
      .removeOcg(data.mapID, _ocg.id)
      .then(() => {
        toastStore.trigger({
          message: maps.successDeleteOcgMsg(_ocg.display_name),
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
</script>

<ConfigLayout title={`Map "${data.mapDisplayName}"`} back={['Maps', maps.listUrl()]}>
  <MapEditorTab mapID={data.mapID} curMode="ocg" />
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
          <th>Create Time</th>
          <th>Operation</th>
        </tr>
      </thead>
      <tbody>
        {#each data.ocgs as ocg, i}
          <tr>
            <td>{i + 1 + data.limit * (data.page - 1)}</td>
            <td>
              {ocg.display_name}
            </td>
            <td>
              {timeDisplay(ocg.created)}
            </td>
            <td>
              <Perm perms="systemx.change_mapocg">
                <a
                  href={maps.ocgEditUrl(data.mapID, ocg.id)}
                  class="variant-filled btn mb-1 w-7 rounded p-1"
                  tip-title="Edit">
                  <i class="fa-solid fa-pencil"></i>
                </a>
              </Perm>
              <Perm perms="systemx.delete_mapocg">
                <button
                  type="button"
                  class="variant-filled btn mb-1 w-7 rounded p-1"
                  on:click={onDeleteCB(ocg)}
                  tip-title="Delete">
                  <i class="fa-solid fa-times"></i>
                </button>
              </Perm>
            </td>
          </tr>
        {:else}
          <tr>
            <td colspan="4">No raw maps yet.</td>
          </tr>
        {/each}
      </tbody>
    </table>
  </div>
</ConfigLayout>
