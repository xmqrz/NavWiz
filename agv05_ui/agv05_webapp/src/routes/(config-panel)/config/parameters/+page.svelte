<script>
  import { getModalStore, getToastStore } from '@skeletonlabs/skeleton';
  import { invalidateAll } from '$app/navigation';

  import parameters from '$lib/shared/services/config/parameters';
  import ConfigLayout from 'components/ConfigLayout.svelte';
  import Perm from 'components/Perm.svelte';

  export let data;
  const modalStore = getModalStore();
  const toastStore = getToastStore();

  function onDeleteCB(key) {
    return function () {
      const modal = {
        type: 'confirm',
        title: 'Delete parameter',
        body: `Are you sure you want to delete "${key}"?`,
        response: (r) => {
          if (!r) {
            return;
          }
          removeParameter(key);
        }
      };
      modalStore.trigger(modal);
    };
  }

  function removeParameter(key) {
    parameters
      .remove(key)
      .then(() => {
        toastStore.trigger({
          message: parameters.successDeleteMsg(key),
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

  function onResetCB(key) {
    return function () {
      const modal = {
        type: 'confirm',
        title: 'Reset parameter',
        body: `Are you sure you want to reset "${key}"?`,
        response: (r) => {
          if (!r) {
            return;
          }
          resetParameter(key);
        }
      };
      modalStore.trigger(modal);
    };
  }

  function resetParameter(key) {
    parameters
      .reset(key)
      .then(() => {
        toastStore.trigger({
          message: parameters.successResetMsg(key),
          timeout: 3000,
          hoverable: true
        });
      })
      .catch((e) => {
        console.log(e);
        // TODO: handle provide error message.
      });
  }
</script>

<ConfigLayout title="Parameters" validation={false}>
  <h2 class="text-2xl font-semibold">
    <i class="fa-solid fa-caret-right"></i> Active Components
  </h2>
  <h3 class="pt-3 text-lg"><i class="fa-solid fa-gamepad"></i> Controllers</h3>
  <div class="table-container pt-3">
    <table class="table table-hover">
      <thead>
        <tr>
          <th width="40px">#</th>
          <th>Component</th>
          <th>Operation</th>
        </tr>
      </thead>
      <tbody>
        {#each data.parameters.active_components.controllers as p, i}
          <tr>
            <td>{i + 1}</td>
            <td>{p.key}</td>
            <td>
              <Perm perms="system.change_parameter">
                <a
                  href={parameters.editUrl(p.key)}
                  class="variant-filled btn mb-1 w-7 rounded p-1"
                  tip-title="Edit">
                  <i class="fa-solid fa-pencil"></i>
                </a>
                <button
                  type="button"
                  class="variant-filled btn mb-1 w-7 rounded p-1"
                  on:click={onResetCB(p.key)}
                  tip-title="Reset to defaults">
                  <i class="fa-solid fa-eraser"></i>
                </button>
              </Perm>
            </td>
          </tr>
        {:else}
          <tr>
            <td colspan="3">No active components.</td>
          </tr>
        {/each}
      </tbody>
    </table>
  </div>
  <h3 class="pt-3 text-lg"><i class="fa-solid fa-truck"></i> Hardware</h3>
  <div class="table-container pt-3">
    <table class="table table-hover">
      <thead>
        <tr>
          <th width="40px">#</th>
          <th>Component</th>
          <th>Operation</th>
        </tr>
      </thead>
      <tbody>
        {#each data.parameters.active_components.hardware as p, i}
          <tr>
            <td>{i + 1}</td>
            <td>{p.key}</td>
            <td>
              <Perm perms="system.change_parameter">
                <a
                  href={parameters.editUrl(p.key)}
                  class="variant-filled btn mb-1 w-7 rounded p-1"
                  tip-title="Edit">
                  <i class="fa-solid fa-pencil"></i>
                </a>
                <button
                  type="button"
                  class="variant-filled btn mb-1 w-7 rounded p-1"
                  on:click={onResetCB(p.key)}
                  tip-title="Reset to defaults">
                  <i class="fa-solid fa-eraser"></i>
                </button>
              </Perm>
            </td>
          </tr>
        {:else}
          <tr>
            <td colspan="3">No active components.</td>
          </tr>
        {/each}
      </tbody>
    </table>
  </div>
  <h2 class="pt-6 text-2xl font-semibold">
    <i class="fa-solid fa-caret-right"></i> Offline Components
  </h2>
  <h3 class="pt-3 text-lg"><i class="fa-solid fa-gamepad"></i> Controllers</h3>
  <div class="table-container pt-3">
    <table class="table table-hover">
      <thead>
        <tr>
          <th width="40px">#</th>
          <th>Component</th>
          <th>Operation</th>
        </tr>
      </thead>
      <tbody>
        {#each data.parameters.offline_components.controllers as p, i}
          <tr>
            <td>{i + 1}</td>
            <td>{p.key}</td>
            <td>
              <Perm perms="system.change_parameter">
                <a
                  href={parameters.editUrl(p.key)}
                  class="variant-filled btn mb-1 w-7 rounded p-1"
                  tip-title="Edit">
                  <i class="fa-solid fa-pencil"></i>
                </a>
              </Perm>
              <Perm perms="system.delete_parameter">
                <button
                  type="button"
                  class="variant-filled btn mb-1 w-7 rounded p-1"
                  on:click={onDeleteCB(p.key)}
                  tip-title="Delete">
                  <i class="fa-solid fa-times"></i>
                </button>
              </Perm>
            </td>
          </tr>
        {:else}
          <tr>
            <td colspan="3">No offline components.</td>
          </tr>
        {/each}
      </tbody>
    </table>
  </div>
  <h3 class="pt-3 text-lg"><i class="fa-solid fa-truck"></i> Hardware</h3>
  <div class="table-container pt-3">
    <table class="table table-hover">
      <thead>
        <tr>
          <th width="40px">#</th>
          <th>Component</th>
          <th>Operation</th>
        </tr>
      </thead>
      <tbody>
        {#each data.parameters.offline_components.hardware as p, i}
          <tr>
            <td>{i + 1}</td>
            <td>{p.key}</td>
            <td>
              <Perm perms="system.change_parameter">
                <a
                  href={parameters.editUrl(p.key)}
                  class="variant-filled btn mb-1 w-7 rounded p-1"
                  tip-title="Edit">
                  <i class="fa-solid fa-pencil"></i>
                </a>
              </Perm>
              <Perm perms="system.delete_parameter">
                <button
                  type="button"
                  class="variant-filled btn mb-1 w-7 rounded p-1"
                  on:click={onDeleteCB(p.key)}
                  tip-title="Delete">
                  <i class="fa-solid fa-times"></i>
                </button>
              </Perm>
            </td>
          </tr>
        {:else}
          <tr>
            <td colspan="3">No offline components.</td>
          </tr>
        {/each}
      </tbody>
    </table>
  </div>
</ConfigLayout>
