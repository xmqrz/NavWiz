<script>
  import { getModalStore } from '@skeletonlabs/skeleton';

  import KeyboardBackdrop from 'components/keyboard/KeyboardBackdrop.svelte';
  import Input from 'components/Input.svelte';

  // Props
  export let parent;

  const modalStore = getModalStore();

  const authenticationOptions = [
    { value: 'peap', label: 'Secure Password (PEAP)' },
    { value: 'tls', label: 'Certificate (EAP-TLS)' }
  ];
  let data_hssid = {
    hssid: '',
    encrypt: 'WPA/WPA2-PSK',
    identity: '',
    password: '',
    eap: 'peap'
  };
  const encryptionOptions = ['WPA/WPA2-PSK', 'WPA/WPA2-EAP'];

  let passVisibility = false;

  function onConnect() {
    $modalStore[0].response({
      ssid: data_hssid.hssid,
      identity: data_hssid.identity || '',
      password: data_hssid.password || '',
      encrypt: data_hssid.encrypt || '',
      eap: data_hssid.eap || ''
    });
    modalStore.close();
  }

  function onCancel() {
    $modalStore[0].response();
    modalStore.close();
  }

  function toggleHssidPass() {
    passVisibility = !passVisibility;
  }
</script>

{#if $modalStore[0]}
  <KeyboardBackdrop>
    <div class="card flex max-h-[90vh] w-96 flex-col space-y-2 self-center p-4">
      <header>
        <div align="center">
          <span class="text-lg">Add Network</span>
        </div>
        <hr />
      </header>
      <form name="hiddenSsidForm" class="grow overflow-auto">
        <div>
          <label>
            <span>SSID</span>
            <input
              class="keyboard input"
              name="hssid"
              type="text"
              bind:value={data_hssid.hssid}
              autocomplete="off" />
          </label>
        </div>
        <div>
          <label>
            <span>Encryption</span>
            <select class="select rounded-token" bind:value={data_hssid.encrypt} required>
              {#each encryptionOptions as enc}
                <option value={enc}>
                  {enc}
                </option>
              {/each}
            </select>
          </label>
        </div>
        {#if data_hssid.encrypt.includes('EAP')}
          <div>
            <label>
              <span>Authentication Method: </span>
              <select class="select rounded-token" bind:value={data_hssid.eap} required>
                {#each authenticationOptions as authenciation}
                  <option value={authenciation.value}>
                    {authenciation.label}
                  </option>
                {/each}
              </select>
            </label>
          </div>
          <div>
            <label>
              <span>Identity: </span>
              <input
                class="keyboard input"
                name="hssid-identity"
                type="text"
                bind:value={data_hssid.identity}
                autocomplete="off" />
            </label>
          </div>
        {/if}
        {#if (data_hssid.eap == 'peap' && data_hssid.encrypt.includes('EAP')) || data_hssid.encrypt.includes('PSK')}
          <div>
            <p class="flex justify-between">Password:</p>
            <label>
              <div class="input-group input-group-divider grid-cols-[1fr_auto]">
                <Input
                  class="keyboard input"
                  type={passVisibility ? 'text' : 'password'}
                  bind:value={data_hssid.password} />
                <button
                  type="button"
                  on:touchstart|preventDefault|stopPropagation={toggleHssidPass}
                  on:mousedown|preventDefault|stopPropagation={toggleHssidPass}>
                  <i class="fa-solid fa-eye"></i>
                </button>
              </div>
            </label>
          </div>
        {/if}
      </form>
      <footer class="modal-footer {parent.regionFooter}">
        <button type="button" class="variant-filled-error btn" on:click={onCancel}>
          Cancel
        </button>
        <button type="button" class="variant-filled-success btn" on:click={onConnect}>
          Connect
        </button>
      </footer>
    </div>
  </KeyboardBackdrop>
{/if}
