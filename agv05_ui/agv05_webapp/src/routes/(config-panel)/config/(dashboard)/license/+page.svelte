<script>
  import { TabGroup, Tab } from '@skeletonlabs/skeleton';
  import ConfigLayout from 'components/ConfigLayout.svelte';

  import Offline from './Offline.svelte';
  import Online from './Online.svelte';
  import Return from './Return.svelte';

  export let data;

  const features = [
    [1 << 1, 'Runs on physical machine'],
    [1 << 2, 'Runs on virtual machine'],
    [1 << 16, 'Tracked mode'],
    [1 << 17, 'Trackless mode'],
    [1 << 18, 'Dynamic obstacle avoidance']
  ];
  const r = data.licenseInfo;
  const req = data.req;

  function df(t) {
    return new Date(t * 1000).toLocaleString();
  }

  const a_placeholder = 'blob:navwiz.lrt';
  function download(evt) {
    let a = evt.target;

    let blob = new Blob([req.license_ret], { type: 'octet/stream' });
    let url = window.URL.createObjectURL(blob);

    a.href = url;

    setTimeout(() => {
      a.href = a_placeholder;
      window.URL.revokeObjectURL(url);
    }, 150);
  }

  let tab = 0;
</script>

<ConfigLayout title="NavWiz License">
  <div class="mb-4 space-y-2">
    <h3 class="mb-2 text-3xl">License Info</h3>
    <p>Machine ID: {req.machine_id}</p>
    {#if r.features}
      <p>Registered Owner: {r.owner}</p>
      <p>Registered Email: {r.email}</p>
      {#if r.days >= 0}
        <p>
          License Type:
          {#if !(r.features & 1)}
            Trial ({r.days} day(s) remaining)
          {:else if r.days}
            Subscription (renews on {df(r.till)})
          {:else}
            Permanent
          {/if}
        </p>
        {#if !r.valid}
          <p class="font-semibold text-red-500">Some features are not licensed.</p>
        {/if}
      {:else if r.features & 1}
        <p class="font-semibold text-red-500">
          Your subscription license has expired on {df(r.till)}. Please renew your license.
        </p>
      {:else}
        <p class="font-semibold text-red-500">
          Your trial license has expired on {df(r.till)}. Please consider purchasing a new
          license.
        </p>
      {/if}
      <p>License Bundle:</p>
      <ul class="pl-8">
        {#each features as [flag, feature]}
          <li>
            {#if r.features & flag}
              <i class="fa-solid fa-check-circle mr-2 text-green-500"></i>
            {:else}
              <i class="fa-solid fa-times-circle mr-2 text-red-500"></i>
            {/if}
            {feature}
          </li>
        {/each}
      </ul>
    {:else}
      <p class="font-semibold text-red-500">The license key is invalid.</p>
    {/if}
    {#if req.license_ret}
      <div class="pl-2">
        <a class="anchor" href={a_placeholder} download="navwiz.lrt" on:click={download}>
          <i class="fa fa-download"></i> Download license return file
        </a>
      </div>
    {/if}
  </div>

  <TabGroup>
    <Tab bind:group={tab} name="online" value={0}>Online Activation</Tab>
    <Tab bind:group={tab} name="offline" value={1}>Offline Activation</Tab>
    {#if r.features}
      <Tab bind:group={tab} name="return" value={2}>License Return</Tab>
    {/if}
    <svelte:fragment slot="panel">
      {#if tab === 0}
        <Online />
      {:else if tab === 1}
        <Offline {req} />
      {:else if tab === 2}
        <Return />
      {/if}
    </svelte:fragment>
  </TabGroup>
</ConfigLayout>
