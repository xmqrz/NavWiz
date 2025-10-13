<script>
  import ConfigLayout from 'components/ConfigLayout.svelte';
  import softwareUpdate from '$lib/shared/services/config/software-update';
  import { goto } from '$app/navigation';

  const srcChoices = [['usb', 'USB pendrive']];
  let step = 0;
  let source = srcChoices[0][0];
  let usb;
  let usbChoices = [];
  let usbReq = false;
  let usbVerReq = false;
  let usbUpdateReq = false;
  let usbVerChoices = [];
  let currentVersion = '';
  let kioskInstalled = true;
  let kioskVal = true;
  let versionVal;

  function onSourceSubmit() {
    step = 1;
  }

  function onUsbSubmit() {
    if (!usb || usbVerReq) {
      return;
    }
    usbVerReq = true;
    step = 2;
    softwareUpdate
      .getUsbVersion(usb)
      .then((d) => {
        usbVerChoices = d.versions.map((v) => [v, v]);
        currentVersion = d.current_version;
        kioskInstalled = d.kiosk_installed;
      })
      .finally(() => {
        usbVerReq = false;
      });
  }

  function onUsbUpdateSubmit() {
    if (!usb || !versionVal || usbUpdateReq) {
      return;
    }
    usbUpdateReq = true;
    softwareUpdate
      .usbUpdate(usb, versionVal, kioskVal)
      .then((d) => {
        goto('/config/software-update-progress', {
          state: {
            message: softwareUpdate.successUpdateMsg(),
            logFile: d.log_file
          }
        });
      })
      .finally(() => {
        usbUpdateReq = false;
      });
  }

  function updateUsbChoices() {
    if (usbReq) {
      return;
    }
    usbReq = true;

    softwareUpdate
      .getUsb()
      .then((d) => {
        if (!d || !d.actions || !d.actions.POST) {
          return;
        }
        usbChoices = d.actions.POST.usb.choices.map((c) => [c.value, c.display_name]);
      })
      .finally(() => {
        usbReq = false;
      });
  }

  function onStepChange(s) {
    if (s === 1) {
      updateUsbChoices();
    }
  }

  $: onStepChange(step);
</script>

<ConfigLayout title="Check for Updates" validation={false}>
  {@const formLabelClass = 'text-lg font-semibold pr-4 text-right'}

  <div class="flex flex-col space-y-4">
    <div class="text-xl">{`Step ${step + 1} of 3`}</div>
    {#if step === 0}
      <div class="container grid grid-cols-4 space-y-4">
        <label class="col-span-4 grid grid-cols-4">
          <span class={formLabelClass}>Source</span>
          <div class="col-span-3">
            <select class="select px-7 rounded-token" bind:value={source}>
              {#each srcChoices as c}
                <option value={c[0]}>{c[1]}</option>
              {/each}
            </select>
          </div>
        </label>
        <div class="col-span-3 col-start-2">
          <button type="button" class="variant-filled-primary btn" on:click={onSourceSubmit}>
            Submit
          </button>
        </div>
      </div>
    {:else if step === 1}
      <div>
        <button
          type="button"
          class="variant-ghost btn btn-sm"
          on:click={() => {
            step = 0;
          }}>
          <i class="fa-solid fa-chevron-left mr-2"></i>
          Previous Step
        </button>
      </div>
      <div class="container grid grid-cols-4 space-y-4">
        <label class="col-span-4 grid grid-cols-4">
          <span class={formLabelClass}>Usb device</span>
          <div class="col-span-3">
            <div class="input-group input-group-divider grid-cols-[1fr_auto]">
              <select class="select px-7 rounded-token" bind:value={usb}>
                {#each usbChoices as c}
                  <option value={c[0]}>{c[1]}</option>
                {/each}
              </select>
              <button type="button" class="btn" on:click={updateUsbChoices}>
                <i class="fa-solid fa-refresh"></i>
              </button>
            </div>
          </div>
        </label>
        <div class="col-span-3 col-start-2">
          <button type="button" class="variant-filled-primary btn" on:click={onUsbSubmit}>
            Submit
          </button>
        </div>
      </div>
    {:else if step === 2}
      <div>
        <button
          type="button"
          class="variant-ghost btn btn-sm"
          on:click={() => {
            step = 1;
          }}>
          <i class="fa-solid fa-chevron-left mr-2"></i>
          Previous Step
        </button>
      </div>
      <div class="container grid grid-cols-4 space-y-4">
        <div class="col-span-4 grid grid-cols-4">
          <span class={formLabelClass}>Current Version</span>
          <div class="col-span-3">
            <span>{currentVersion || '-'}</span>
          </div>
        </div>
        <label class="col-span-4 grid grid-cols-4">
          <span class={formLabelClass}>Update Version</span>
          <div class="col-span-3">
            <div class="input-group input-group-divider grid-cols-[1fr_auto]">
              <select class="select px-7 rounded-token" bind:value={versionVal}>
                {#each usbVerChoices as c}
                  <option value={c[0]}>{c[1]}</option>
                {/each}
              </select>
              <button type="button" class="btn" on:click={updateUsbChoices}>
                <i class="fa-solid fa-refresh"></i>
              </button>
            </div>
          </div>
          <span class="col-span-3 col-start-2 text-gray-500"
            >Select which version to update to.</span>
        </label>
        {#if !kioskInstalled}
          <label class="col-span-4 grid grid-cols-4">
            <div class="col-span-3 col-start-2 flex items-center space-x-2">
              <input class="checkbox mr-1" type="checkbox" bind:checked={kioskVal} />
              <span class={formLabelClass}>Install Kiosk</span>
            </div>
            <span class="col-span-3 col-start-2 text-gray-500"
              >Kiosk is missing. Reinstall kiosk?</span>
          </label>
        {/if}
        <div class="col-span-3 col-start-2">
          <button
            type="button"
            class="variant-filled-primary btn"
            on:click={onUsbUpdateSubmit}>
            Submit
          </button>
        </div>
      </div>
    {/if}
  </div>
</ConfigLayout>
