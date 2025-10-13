<script>
  import { Paginator, getModalStore, getToastStore } from '@skeletonlabs/skeleton';
  import { goto, invalidateAll } from '$app/navigation';
  import inView from 'actions/in-view.js';
  import ConfigLayout from 'components/ConfigLayout.svelte';
  import Perm from 'components/Perm.svelte';
  import webhooks from '$lib/shared/services/config/webhooks';

  export let data;
  const modalStore = getModalStore();
  const toastStore = getToastStore();

  let addVisible = false;

  function onPageChange(e) {
    const page = e.detail + 1;
    goto(webhooks.listUrl(page));
  }

  function onDeleteCB(item) {
    modalStore.trigger({
      type: 'confirm',
      title: 'Delete Webhook',
      body: `Are you sure you want to delete webhook "${item.url}"?`,
      response: (r) => {
        if (r) {
          webhooks
            .remove(item.id)
            .then(() => {
              toastStore.trigger({
                message: webhooks.successDeleteMsg(item.url),
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
      }
    });
  }
</script>

<ConfigLayout title="Webhooks" validation={false}>
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
          <th>URL</th>
          <th>Events</th>
          <th>SSL Verification</th>
          <th>Operation</th>
        </tr>
      </thead>
      <tbody>
        {#each data.items as item, i}
          <tr>
            <td>{i + 1 + data.limit * (data.page - 1)}</td>
            <td>
              {item.url}
            </td>
            <td>
              {#each item.events.split('","') as evt}
                <span class="variant-filled-surface chip float-left ml-2">
                  {evt.replaceAll('"', '')}
                </span>
              {/each}
            </td>
            <td>
              {#if item.verify_ssl}
                <i class="fa-solid fa-check-circle m-auto text-xl text-green-500"></i>
              {:else}
                <i class="fa-solid fa-times-circle m-auto text-xl text-red-500"></i>
              {/if}
            </td>
            <td>
              <Perm perms={['system.change_webhook']}>
                <a
                  href={webhooks.editUrl(item.id)}
                  tip-title="Edit"
                  class="variant-filled btn mb-1 w-7 rounded p-1">
                  <i class="fa-solid fa-pencil"></i>
                </a>
              </Perm>
              <Perm perms={['system.delete_webhook']}>
                <button
                  type="button"
                  tip-title="Delete"
                  class="variant-filled btn mb-1 w-7 rounded p-1"
                  on:click={() => onDeleteCB(item)}>
                  <i class="fa-solid fa-times"></i>
                </button>
              </Perm>
            </td>
          </tr>
        {:else}
          <tr>
            <td colspan="5">No webhooks yet.</td>
          </tr>
        {/each}
      </tbody>
    </table>
  </div>
  <Perm perms={['system.add_webhook']}>
    <div
      class="mt-5"
      use:inView
      on:enter={() => (addVisible = true)}
      on:exit={() => (addVisible = false)}>
      <a
        href={webhooks.addUrl()}
        class="variant-filled-secondary btn p-5 text-white"
        tip-title="Add new webhook">
        <i class="fa-solid fa-plus"></i>
      </a>
    </div>
    {#if !addVisible}
      <div class="variant-glass-surface fixed bottom-0 left-0 w-full">
        <a
          href={webhooks.addUrl()}
          class="variant-filled-secondary btn my-3 ml-5 p-5 text-white"
          tip-title="Add new webhook">
          <i class="fa-solid fa-plus"></i>
        </a>
      </div>
    {/if}
  </Perm>
</ConfigLayout>
