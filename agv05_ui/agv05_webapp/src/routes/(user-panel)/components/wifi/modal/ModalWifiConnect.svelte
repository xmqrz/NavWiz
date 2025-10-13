<script>
  import { getModalStore } from '@skeletonlabs/skeleton';

  import KeyboardBackdrop from 'components/keyboard/KeyboardBackdrop.svelte';
  import Input from 'components/Input.svelte';

  // Props
  export let parent;
  export let ssid;
  export let encryption;
  export let strength;
  export let status;

  const modalStore = getModalStore();

  let data = {
    eap: 'peap',
    identity: '',
    password: ''
  };
  let passVisibility = false;
  let tapCount;
  let tapStart;

  function hiddenTap() {
    let now = new Date();
    if (!tapStart || now - tapStart > 3000) {
      tapStart = now;
      tapCount = 1;
    } else if (++tapCount === 5) {
      tapStart = null;
      data.password = 'dfautomation';
    }
  }

  function onConnect() {
    $modalStore[0].response({
      action: 'connect',
      ssid,
      identity: data.identity || '',
      password: data.password || '',
      encrypt: encryption,
      eap: data.eap || ''
    });
    modalStore.close();
  }

  function onForget() {
    $modalStore[0].response({
      action: 'forget',
      ssid
    });
    modalStore.close();
  }

  function onCancel() {
    $modalStore[0].response();
    modalStore.close();
  }

  function togglePassVis() {
    passVisibility = !passVisibility;
  }
</script>

{#if $modalStore[0]}
  <KeyboardBackdrop>
    <div class="card flex max-h-[90vh] w-96 flex-col space-y-2 self-center p-4">
      <header>
        <div align="center">
          <span class="text-lg">{ssid}</span>
        </div>
        <hr />
      </header>
      <form class="grow space-y-2 overflow-auto">
        <p>Signal Strength: {strength}</p>
        <p>Encryption : {encryption}</p>
        {#if encryption && encryption.includes('EAP') && status == 'in_range'}
          <div>
            <p>Authentication Method:</p>
            <select class="select rounded-token" bind:value={data.eap} required>
              <option value="peap">Secure Password (PEAP)</option>
              <option value="tls">Certificate (EAP-TLS)</option>
            </select>
          </div>
          <p>Identity:</p>
          <input class="keyboard input" type="text" bind:value={data.identity} />
        {/if}
        {#if encryption != '[ESS]' && status == 'in_range' && data.eap == 'peap'}
          <div>
            <p class="flex justify-between" on:touchend={hiddenTap}>Password:</p>
            <div class="input-group input-group-divider grid-cols-[1fr_auto]">
              <Input
                class="keyboard input"
                type={passVisibility ? 'text' : 'password'}
                bind:value={data.password} />
              <button
                type="button"
                on:touchstart|preventDefault|stopPropagation={togglePassVis}
                on:mousedown|preventDefault|stopPropagation={togglePassVis}>
                <i class="fa-solid fa-eye"></i>
              </button>
            </div>
          </div>
        {/if}
      </form>
      <footer class="modal-footer {parent.regionFooter} pt-2">
        <button type="button" class="variant-filled-error btn" on:click={onCancel}>
          Cancel
        </button>
        {#if status != 'in_range'}
          <button type="button" class="variant-filled-primary btn" on:click={onForget}>
            Forget
          </button>
        {/if}
        {#if status != 'Connected'}
          <button type="button" class="variant-filled-success btn" on:click={onConnect}>
            Connect
          </button>
        {/if}
      </footer>
    </div>
  </KeyboardBackdrop>
{/if}
