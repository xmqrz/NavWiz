<script>
  import ConfigLayout from 'components/ConfigLayout.svelte';
  import { FileButton } from '@skeletonlabs/skeleton';
  import { goto } from '$app/navigation';

  import plugin from '$lib/shared/services/config/hardware-plugin';

  export let data;

  let current = data.plugins;
  let errors = {};

  let ordering = [...Array(5).keys()];
  let clearing = [];
  let fileInputs = [];
  let files = [];
  let uploading = false;

  function onSubmit() {
    let data = new FormData();

    for (let i = 0; i < ordering.length; i++) {
      let o = ordering[i];
      if (files[o]) {
        data.append(`plugins[${i}]upload`, files[o][0]);
      } else {
        data.append(`plugins[${i}]name`, (!clearing[o] && current[o]) || '');
      }
    }

    plugin
      .update(data)
      .then((_v) => {
        uploading = false;
        goto('/', { invalidateAll: true });
      })
      .catch((e) => {
        uploading = false;
        if (e.errorCode === 400) {
          const response = e.cause;
          response
            .json()
            .then(function (err) {
              errors = err;
              if (errors.plugins) {
                let e = [];
                for (let i in errors.plugins) {
                  let o = ordering[i];
                  e[o] = errors.plugins[i];
                }
                errors.plugins = e;
              }
            })
            .catch(function () {
              errors = { detail: e.message };
            });
          return;
        }
        errors = { detail: e.message };
      });

    errors = {};
    uploading = true;
  }

  function hwDownloadUrl(fn) {
    return `${API_URL}/config/hardware-plugin/${fn}/download`;
  }
</script>

<ConfigLayout title="Manage Hardware Plugin" validation={false}>
  {#if errors.detail}
    <div class="alert variant-filled-error mb-3">
      <div class="alert-message">{errors.detail}</div>
    </div>
  {/if}
  <form class="w-full space-y-3 lg:w-3/4" on:submit|preventDefault={onSubmit}>
    <div class="label">
      <span class="text-lg font-semibold">Plugin files</span>
    </div>
    <div>
      Arrange the plugin files in the order from top to bottom layer. Upper layer overwrites
      the content of the layers below it.
    </div>
    {#each ordering as o, idx (o)}
      <div class="flex max-w-3xl flex-row items-center pl-7">
        <FileButton
          name="plugin_{idx + 1}"
          class="mr-2 inline-block"
          accept=".plugin"
          bind:fileInput={fileInputs[o]}
          bind:files={files[o]} />
        <div class="flex flex-auto flex-col">
          <div>
            <span class="font-semibold">Current:</span>
            {#if current[o]}
              <a href={hwDownloadUrl(current[o])} class="anchor">
                {current[o]}
              </a>
              <label class="label inline-block pl-5">
                <input class="checkbox" type="checkbox" bind:checked={clearing[o]} />
                <span class="font-semibold">Clear</span>
              </label>
            {:else}
              -
            {/if}
          </div>
          <div>
            <span class="font-semibold">Change:</span>
            {files[o]?.[0].name || '-'}
            {#if files[o]}
              <button
                type="button"
                class="bg-initial btn py-0"
                on:click={() => {
                  fileInputs[o].value = '';
                  files[o] = null;
                }}>
                <i class="fa fa-close"></i>
              </button>
            {/if}
          </div>
          <div class="text-error-500">
            {#each errors?.plugins?.[o]?.name || [] as e}
              <div>{e}</div>
            {/each}
            {#each errors?.plugins?.[o]?.upload || [] as e}
              <div>{e}</div>
            {/each}
          </div>
        </div>
        <div>
          {#if idx}
            <button
              type="button"
              class="variant-ringed btn"
              on:click={() => {
                let c = ordering[idx];
                ordering[idx] = ordering[idx - 1];
                ordering[idx - 1] = c;
              }}>
              <i class="fa fa-arrow-up"></i>
            </button>
          {/if}
          {#if idx < ordering.length - 1}
            <button
              type="button"
              class="variant-ringed btn"
              on:click={() => {
                let c = ordering[idx];
                ordering[idx] = ordering[idx + 1];
                ordering[idx + 1] = c;
              }}>
              <i class="fa fa-arrow-down"></i>
            </button>
          {:else}
            <button type="button" class="variant-ringed btn invisible">
              <i class="fa fa-arrow-down"></i>
            </button>
          {/if}
        </div>
      </div>
    {/each}
    <div class="px-3 pt-7">
      <button type="submit" class="variant-filled-primary btn" class:disabled={uploading}>
        {#if uploading}
          Uploading... Please wait...
        {:else}
          Update and Hard Reboot
        {/if}
      </button>
    </div>
  </form>
</ConfigLayout>
