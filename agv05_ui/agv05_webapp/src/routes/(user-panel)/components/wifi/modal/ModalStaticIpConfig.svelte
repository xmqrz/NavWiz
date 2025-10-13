<script>
  import { getModalStore } from '@skeletonlabs/skeleton';

  import { ipValidator, netmaskValidator } from '$lib/shared/validators/validators.js';
  import { createFieldValidator } from '$lib/shared/validators/validation.js';
  import KeyboardBackdrop from 'components/keyboard/KeyboardBackdrop.svelte';

  const [validityIp, validateIp] = createFieldValidator(ipValidator());
  const [validityNetmask, validateNetmask] = createFieldValidator(netmaskValidator());
  const [validityGateway, validateGateway] = createFieldValidator(ipValidator());
  const [validityDns, validateDns] = createFieldValidator(ipValidator());

  // Props
  export let parent;
  export let config;

  const modalStore = getModalStore();

  const cForm =
    'grow overflow-auto border border-surface-500 p-2 space-y-2 rounded-container-token';

  function onFormSubmit() {
    $modalStore[0].response($config);
    modalStore.close();
  }

  function onCancel() {
    $modalStore[0].response();
    modalStore.close();
  }
</script>

{#if $modalStore[0]}
  <KeyboardBackdrop>
    <div class="card flex max-h-[90vh] w-96 flex-col space-y-2 self-center p-4">
      <header>
        <div align="center">
          <span class="text-lg">IP Setting</span>
        </div>
        <hr />
      </header>
      <label class="px-2 pb-5">
        <span>Static IPv4</span>
        <div class="float-right">
          <input type="checkbox" class="checkbox" bind:checked={$config.staticOpt} />
        </div>
      </label>
      {#if $config.staticOpt}
        <form name="staticIpConfigForm" class={cForm}>
          <div>
            <label>
              IP Address
              <input
                name="ipv4"
                class="keyboard input"
                data-keyboard="ip"
                type="text"
                bind:value={$config.ipv4}
                placeholder="0.0.0.0"
                autocomplete="off"
                required
                class:input-error={!$validityIp.valid}
                use:validateIp={$config.ipv4} />
            </label>
          </div>
          <div>
            <label>
              Netmask
              <input
                name="netmask"
                class="keyboard input"
                data-keyboard="ip"
                type="text"
                bind:value={$config.netmask}
                required
                placeholder="0.0.0.0"
                autocomplete="off"
                class:input-error={!$validityNetmask.valid}
                use:validateNetmask={$config.netmask} />
            </label>
          </div>
          <div>
            <label>
              Gateway
              <input
                name="gateway"
                class="keyboard input"
                data-keyboard="ip"
                type="text"
                bind:value={$config.gateway}
                placeholder="0.0.0.0"
                autocomplete="off"
                required
                class:input-error={!$validityGateway.valid}
                use:validateGateway={$config.gateway} />
            </label>
          </div>
          <div>
            <label>
              DNS Nameserver
              <input
                name="dns-nameserver"
                class="keyboard input"
                data-keyboard="ip"
                type="text"
                bind:value={$config.dnsNameserver}
                placeholder="0.0.0.0"
                autocomplete="off"
                required
                class:input-error={!$validityDns.valid}
                use:validateDns={$config.dnsNameserver} />
            </label>
          </div>
        </form>
      {/if}
      <footer class="modal-footer {parent.regionFooter}">
        <button type="button" class="btn {parent.buttonNeutral}" on:click={onCancel}>
          {parent.buttonTextCancel}
        </button>
        <button
          type="button"
          class="btn {parent.buttonPositive}"
          on:click={onFormSubmit}
          disabled={!$validityDns.valid ||
            !$validityGateway.valid ||
            !$validityIp.valid ||
            !$validityNetmask.valid}>Apply</button>
      </footer>
    </div>
  </KeyboardBackdrop>
{/if}
