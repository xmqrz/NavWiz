<script>
  import { getContext, onMount } from 'svelte';
  import { getAPI } from '$lib/utils';

  const handleIdentify = getContext('networkIdentify');
  const identify = {
    name: '',
    count: 0,
    interval: null
  };

  let data = {
    hostname: '-',
    scheme: 'HTTP',
    interfaces: [],
    default_interface: '-',
    dns_nameservers: []
  };

  onMount(() => {
    const interval = setInterval(updateData, 3000);
    updateData();

    return () => {
      clearInterval(interval);
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

  const updateData = async () => {
    data = await getAPI('/config/network/status');
  };

  const dateToInternalDate = (x) => {
    const d = new Date(x);
    return `${d.getFullYear()}-${String(d.getMonth() + 1).padStart(2, '0')}-${String(
      d.getDate()
    ).padStart(2, '0')}`;
  };
</script>

<div class="container w-full space-y-4 lg:w-3/4">
  <table class="w-full">
    <tr>
      <th class="w-1/3 text-left">Hostname:</th>
      <td>{data.hostname}</td>
    </tr>
  </table>

  <table class="w-full">
    <tr>
      <th class="w-1/3 text-left">Network Scheme:</th>
      <td>{data.scheme}</td>
    </tr>
    {#if 'crt_exp' in data}
      <tr>
        <td>Certificate Valid Until:</td>
        <td>{dateToInternalDate(data.crt_exp)}</td>
      </tr>
    {/if}
  </table>

  <table class="w-full space-y-4">
    <tr>
      <th class="w-1/3 text-left">Wi-Fi CA Certificate:</th>
      <td>{'wifi_ca_exp' in data ? data.wifi_ca_fingerprint : '-'}</td>
    </tr>
    {#if 'wifi_ca_exp' in data}
      <tr>
        <td>Certificate Valid Until:</td>
        <td>{dateToInternalDate(data.wifi_ca_exp)}</td>
      </tr>
    {/if}
    <tr>
      <th class="text-left">Wi-Fi Certificate:</th>
      <td>{'wifi_crt_exp' in data ? data.wifi_crt_fingerprint : '-'}</td>
    </tr>
    {#if 'wifi_crt_exp' in data}
      <tr>
        <td>Certificate Valid Until:</td>
        <td>{dateToInternalDate(data.wifi_crt_exp)}</td>
      </tr>
    {/if}
  </table>

  {#each data.interfaces as iface}
    {@const connected = iface.link_info[3]}
    <table class="w-full">
      <tr>
        <th class="text-left">
          {iface.name}
          {#if iface.name === data.default_interface}
            (Main Interface)
          {/if}
          {#if !iface.wifi}
            {@const color =
              identify.name == iface.name && identify.count % 2 == 0 ? 'gold' : 'black'}
            <button
              type="button"
              class="variant-filled-tertiary btn rounded"
              class:disabled={identify.name && identify.name != iface.name}
              on:click|preventDefault={() => handleIdentify(iface.name, updateIdentify)}>
              <i class="fa-solid fa-lightbulb" style:color></i>
            </button>
          {/if}
        </th>
      </tr>
    </table>
    <table class="w-full">
      <tr>
        <td class="w-1/3">MAC Address:</td>
        <td>{iface.mac}</td>
      </tr>
      <tr>
        {#if !connected}
          <td />
          <td>Disconnected</td>
        {:else if 'wifi' in iface}
          <td>Wireless:</td>
          <td>{iface.wifi.specs}</td>
        {:else}
          <td>Link Speed:</td>
          <td>{iface.link_info[0]} Mbps</td>
        {/if}
      </tr>
    </table>
    {#if connected}
      {#if iface.wifi}
        <table class="w-full">
          <tr>
            <td class="w-1/12" />
            <td class="w-1/4">SSID:</td>
            <td>{iface.wifi.ssid}</td>
          </tr>
          <tr>
            <td />
            <td>Mode:</td>
            <td>{iface.wifi.mode}</td>
          </tr>
          <tr>
            <td />
            <td>Frequency:</td>
            <td>{iface.wifi.frequency}</td>
          </tr>
          <tr>
            <td />
            <td>Access Point:</td>
            <td>{iface.wifi.access_point}</td>
          </tr>
          <tr>
            <td />
            <td>Link Speed:</td>
            <td>{iface.link_info[0]}</td>
          </tr>
          <tr>
            <td />
            <td>Link Quality:</td>
            <td>{iface.wifi.quality}</td>
          </tr>
          <tr>
            <td />
            <td>Signal Level:</td>
            <td>{iface.wifi.signal_level}</td>
          </tr>
        </table>
      {/if}
      <table class="w-full">
        <tr>
          <td class="w-1/3">IP Address:</td>
          <td>{'ip' in iface ? iface.ip : '-'}</td>
        </tr>
        <tr>
          <td>Subnet Mask:</td>
          <td>{'dotted_quad_netmask' in iface ? iface.dotted_quad_netmask : '-'}</td>
        </tr>
        {#if iface.name === data.default_interface}
          <tr>
            <td>Gateway:</td>
            <td>{'gateway' in data ? data.gateway : '-'}</td>
          </tr>
          <tr>
            <td>DNS Server{data.dns_nameservers.length > 1 ? 's' : ''}:</td>
            <td>{data.dns_nameservers.length ? data.dns_nameservers[0] : '-'}</td>
          </tr>
          {#each data.dns_nameservers as dns, i}
            {#if i}
              <tr>
                <td />
                <td>{dns}</td>
              </tr>
            {/if}
          {/each}
        {/if}
        <tr>
          <td>Bytes Received:</td>
          <td>{'get_stats' in iface ? iface.get_stats.rx_bytes.toLocaleString() : 0}</td>
        </tr>
        <tr>
          <td>Bytes Sent:</td>
          <td>{'get_stats' in iface ? iface.get_stats.tx_bytes.toLocaleString() : 0}</td>
        </tr>
      </table>
    {/if}
  {/each}
</div>
