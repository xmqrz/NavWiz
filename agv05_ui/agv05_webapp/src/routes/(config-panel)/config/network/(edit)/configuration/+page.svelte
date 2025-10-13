<script>
  import { getContext, onMount } from 'svelte';
  import { getAPI } from '$lib/utils';

  const pathname = '/config/network/configuration';
  const handleSubmit = getContext('networkSubmit');
  const handleIdentify = getContext('networkIdentify');
  const identify = {
    name: '',
    count: 0,
    interval: null
  };

  let data = {
    interfaces: []
  };

  onMount(async () => {
    data = await getAPI(pathname);

    return () => {
      updateIdentify(true);
    };
  });

  const updateIdentify = (reset = false, name = '') => {
    if (name == '') {
      if (reset || --identify.count <= 0) {
        identify.name = '';
        identify.count = 0;
        clearInterval(identify.interval);
      }
    } else if (reset) {
      clearInterval(identify.interval);
      identify.interval = setInterval(updateIdentify, 500);
    } else if (identify.count > 0) {
      return false;
    } else {
      identify.name = name;
      identify.count = 20;
    }
    return true;
  };

  function presetInUsed(preset, interface_name) {
    let ret = '';
    data.interfaces.forEach((item) => {
      if (item.source == preset) {
        if (item.name != interface_name) {
          ret = item.name;
        }
        return;
      }
    });
    return ret;
  }

  const formLabelClass = 'text-lg font-semibold pr-4 text-right';
  const formLegendClass = 'text-2xl font-semibold';
</script>

<div class="container">
  <form
    class="w-full lg:w-3/4"
    on:submit|preventDefault={() => handleSubmit({ interfaces: data.interfaces })}>
    {#each data.interfaces as iface}
      {@const color =
        identify.name == iface.name && identify.count % 2 == 0 ? 'gold' : 'black'}
      <fieldset class="space-y-4 p-3 px-10">
        <legend class="legend">
          <span class={formLegendClass}>{iface.name}</span>
          <button
            type="button"
            class="legend variant-filled-tertiary btn rounded"
            class:disabled={identify.name && identify.name != iface.name}
            on:click|preventDefault={() => handleIdentify(iface.name, updateIdentify)}>
            <i class="fa-solid fa-lightbulb" style:color></i>
          </button>
        </legend>
        <hr />
        <label class="grid grid-cols-4">
          <span class={formLabelClass}>Configure IP</span>
          <select
            class="select col-span-3 rounded-token"
            bind:value={iface.source}
            on:change={() => {
              if (iface.source in data.network_preset.field) {
                iface.address = data.network_preset.field[iface.source].address;
                iface.netmask = data.network_preset.field[iface.source].netmask;
                iface.metric = 10;
              }
            }}>
            <option value="">---------</option>
            <option value="dhcp">Automatic (DHCP)</option>
            <option value="static">Manual</option>
            {#each data.network_preset.names as name}
              {#if presetInUsed(name, iface.name) === ''}
                <option value={name}>{data.network_preset.field[name].name}</option>
              {:else}
                <option value={name} disabled>
                  {data.network_preset.field[name].name +
                    ' (' +
                    presetInUsed(name, iface.name) +
                    ')'}
                </option>
              {/if}
            {/each}
          </select>
        </label>
        {#if iface.source != '' && iface.source != 'dhcp'}
          <label class="grid grid-cols-4">
            <span class={formLabelClass}>IP Address</span>
            <input
              required
              class="input col-span-3"
              disabled={iface.source in data.network_preset.field}
              type="text"
              minlength="7"
              maxlength="15"
              pattern="^((\d|[1-9]\d|1\d\d|2[0-4]\d|25[0-5])\.){'{'}3}(\d|[1-9]\d|1\d\d|2[0-4]\d|25[0-5])$"
              placeholder="x.x.x.x"
              title="IP Address x.x.x.x"
              bind:value={iface.address} />
          </label>
          <label class="grid grid-cols-4">
            <span class={formLabelClass}>Subnet Mask</span>
            <input
              required
              class="input col-span-3"
              disabled={iface.source in data.network_preset.field}
              type="text"
              minlength="7"
              maxlength="15"
              pattern="^(((255\.){'{'}3}(25[245]|24[08]|224|192|128))|((255\.){'{'}2}(25[245]|24[08]|224|192|128)\.0)|(255\.(25[245]|24[08]|224|192|128)\.0\.0)|((25[245]|24[08]|224|192|128|0)\.0\.0\.0))$"
              placeholder="x.x.x.x"
              title="Subnet Mask x.x.x.x"
              bind:value={iface.netmask} />
          </label>
        {/if}
        {#if iface.source != ''}
          <label class="grid grid-cols-4">
            <span class={formLabelClass}>Metric</span>
            <input
              required
              class="input col-span-3"
              disabled={iface.source in data.network_preset.field}
              type="number"
              min="1"
              max="10"
              placeholder="1 - 10"
              bind:value={iface.metric} />
            <br />
            <span class="col-span-3 text-gray-500">
              The metric 0 is the highest priority route, but it is reserved for the Wi-Fi
              interface. A larger metric value means a lower priority route.
            </span>
          </label>
        {/if}
      </fieldset>
    {/each}
    <div class="mt-7 grid grid-cols-4 px-10">
      <div class="col-start-2">
        <button type="submit" class="variant-filled-primary btn">Submit</button>
      </div>
    </div>
  </form>
</div>
