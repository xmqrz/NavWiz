<script>
  import { onMount } from 'svelte';
  import ConfigLayout from 'components/ConfigLayout.svelte';
  import { getModalStore, FileButton } from '@skeletonlabs/skeleton';
  import { goto } from '$app/navigation';
  import { getAPI, postAPI } from '$lib/utils';

  const modalStore = getModalStore();
  const form = {
    data: {
      auto_start: false,
      robot_name: '',
      mobile_robot_provider: null,
      simulator_mobile_robot_provider: null,
      network: [],
      env: []
    },
    choices: {},
    file: {
      upload: [],
      download: null
    },
    duplicate: {
      network: [],
      address: [],
      env: []
    },
    error: {}
  };

  function downloadLiveConfig() {
    let blob = new Blob([JSON.stringify(form.data)], { type: 'application/json' });

    let link = document.createElement('a');
    let url = window.URL.createObjectURL(blob);
    link.href = url;
    link.download = 'robot.json';

    link.style.display = 'none';

    document.body.appendChild(link);
    link.click();
    setTimeout(() => {
      window.URL.revokeObjectURL(url);
      document.body.removeChild(link);
    }, 150);
  }

  onMount(async () => {
    const data = await getAPI('/config/robot-config');
    form.data = data.initial;
    form.choices = data.choices;

    handleDuplicate();
  });

  async function handleSubmit() {
    modalStore.trigger({
      type: 'confirm',
      title: 'Robot Configuration',
      body: 'Perform soft reboot and apply robot configuration?',
      response: async (r) => {
        if (r) {
          const res = await postAPI('/config/robot-config', form.data);
          if ('error' in res) {
            form.error = res.error;
          } else {
            goto('/', { invalidateAll: true });
          }
        }
      }
    });
  }

  function handleUp(n, i) {
    if (i) {
      const temp = form.data[n][i - 1];
      form.data[n][i - 1] = form.data[n][i];
      form.data[n][i] = temp;
    }
  }

  function handleDown(n, i) {
    if (i + 1 < form.data[n].length) {
      const temp = form.data[n][i + 1];
      form.data[n][i + 1] = form.data[n][i];
      form.data[n][i] = temp;
    }
  }

  function handleCopy(n, i) {
    form.data[n] = [...form.data[n], JSON.parse(JSON.stringify(form.data[n][i]))];
    handleDuplicate(n);
    if (n === 'network') {
      handleDuplicate('address');
    }
  }

  function handleDelete(n, i) {
    form.data[n].splice(i, 1);
    form.data[n] = form.data[n];
    handleDuplicate(n);
    if (n === 'network') {
      handleDuplicate('address');
    }
  }

  function handleAdd(n) {
    if (n === 'network') {
      form.data.network = [
        ...form.data.network,
        {
          name: '',
          address: '',
          netmask: '',
          extraOptions: {
            autoneg_off: false
          }
        }
      ];
    } else if (n === 'env') {
      form.data.env = [...form.data.env, ['', '']];
    }
  }

  function handleDuplicate(n = null) {
    const set = new Set();
    switch (n) {
      case 'network':
      case 'address':
        // eslint-disable-next-line no-case-declarations
        const key = n === 'network' ? 'name' : n;
        form.duplicate[n] = [];
        form.data.network.forEach((item) => {
          if (set.has(item[key])) {
            form.duplicate[n].push(item[key]);
          } else {
            set.add(item[key]);
          }
        });
        break;
      case 'env':
        form.duplicate[n] = [];
        form.data.env.forEach((item) => {
          if (set.has(item[0])) {
            form.duplicate[n].push(item[0]);
          } else {
            set.add(item[0]);
          }
        });
        break;
      default:
        handleDuplicate('network');
        handleDuplicate('address');
        handleDuplicate('env');
        break;
    }
  }

  const formLabelClass = 'text-lg font-semibold pr-4 text-right';
  const formLegendClass = `text-2xl font-semibold`;
</script>

<ConfigLayout title="Robot Configuration" validation={false}>
  {#if 'error' in form.error}
    <aside class="alert variant-filled-error">
      <!-- Icon -->
      <div><i class="fa-solid fa-triangle-exclamation"></i></div>
      <!-- Message -->
      <div class="alert-message">
        <p>{form.error.error}</p>
      </div>
      <!-- Actions -->
      <div class="alert-actions"></div>
    </aside>
  {/if}

  <div class="mt-5 grid w-full grid-cols-3 lg:grid-cols-4">
    <div class="col-span-2 col-start-2 mb-5">
      <FileButton
        name="robot-config-upload"
        class="float-left"
        button="variant-filled-primary btn m-1"
        accept=".json"
        bind:files={form.file.upload}
        on:change={(event) => {
          if (event.target.files.length === 0) {
            return;
          }

          const alertFileError = (err) => {
            modalStore.trigger({
              type: 'alert',
              title: 'Invalid Robot Config File',
              body: err
            });
          };

          if (event.target.files[0].size > 1024 * 1024) {
            alertFileError('Please select file smaller than 1MB.');
            return;
          }

          let reader = new FileReader();
          reader.onload = (event) => {
            try {
              const cfg = JSON.parse(event.target.result);
              // handle legacy format
              if (cfg.network.length) {
                if (Array.isArray(cfg.network[0])) {
                  cfg.network.forEach((item, index) => {
                    cfg.network[index] = {
                      name: `Preset ${index + 1}`,
                      address: item[0],
                      netmask: item[1],
                      extraOptions: item[2]
                    };
                  });
                }
              }
              form.data = {
                auto_start: cfg.auto_start,
                robot_name: cfg.robot_name,
                mobile_robot_provider: cfg.mobile_robot_provider,
                simulator_mobile_robot_provider: cfg.simulator_mobile_robot_provider,
                network: cfg.network,
                env: cfg.env
              };
              handleDuplicate();
              handleSubmit();
            } catch (err) {
              alertFileError(err.message);
            }
          };
          reader.readAsText(event.target.files[0]);

          // clear file input so that the change event is fired even
          // when the user re-uploads the same file.
          event.target.value = '';
          form.file.upload = [];
        }}>
        <i class="fa-solid fa-upload mr-2"></i>
        Upload
      </FileButton>
      <button
        type="button"
        class="variant-filled-primary btn m-1"
        on:click={downloadLiveConfig}>
        <i class="fa-solid fa-download mr-2"></i>
        Download
      </button>
    </div>
  </div>

  <form on:submit|preventDefault={handleSubmit}>
    <legend class="legend py-5">
      <span class={formLegendClass}>Robot</span>
    </legend>
    <hr />
    <div class="grid w-full grid-cols-3 space-y-6 p-3 pt-5 lg:grid-cols-4">
      <div class="col-span-3 col-start-2">
        <div>
          <label class="flex max-w-max items-center space-x-2">
            <input class="checkbox" type="checkbox" bind:checked={form.data.auto_start} />
            <span class={formLabelClass}>Automatically Start Robot Controller</span>
          </label>
        </div>
      </div>

      <label class="col-span-3 grid grid-cols-3">
        <span class={formLabelClass}>Robot Type</span>
        <div class="col-span-2">
          <select required class="select rounded-token" bind:value={form.data.robot_name}>
            <option value="" />
            <option value="tracked_agv05">Tracked</option>
            <option value="tracked_sim">Tracked (Simulator)</option>
            <option value="trackless_agv05">Trackless</option>
            <option value="trackless_sim">Trackless (Simulator)</option>
            <option value="trackless_simulator">Trackless (Real Simulator)</option>
          </select>
        </div>
      </label>

      {#if ['tracked_agv05', 'trackless_agv05'].includes(form.data.robot_name)}
        <label class="col-span-3 grid grid-cols-3">
          <span class={formLabelClass}>Mobile Robot Capability Provider</span>
          <div class="col-span-2">
            <select
              required
              class="select rounded-token"
              bind:value={form.data.mobile_robot_provider}>
              {#if 'mobile_robot_provider' in form.choices}
                {#each form.choices.mobile_robot_provider as choice}
                  <option value={choice[0]}>{choice[1]}</option>
                {/each}
              {/if}
            </select>
          </div>
        </label>
      {:else if ['trackless_simulator'].includes(form.data.robot_name)}
        <label class="col-span-3 grid grid-cols-3">
          <span class={formLabelClass}>Simulator Mobile Robot Capability Provider</span>
          <div class="col-span-2">
            <select
              required
              class="select rounded-token"
              bind:value={form.data.simulator_mobile_robot_provider}>
              {#if 'simulator_mobile_robot_provider' in form.choices}
                {#each form.choices.simulator_mobile_robot_provider as choice}
                  <option value={choice[0]}>{choice[1]}</option>
                {/each}
              {/if}
            </select>
          </div>
        </label>
      {/if}
    </div>

    <legend class="legend py-5">
      <span class={formLegendClass}>Network Configuration</span>
    </legend>
    <hr />
    <div class="grid grid-cols-3 lg:grid-cols-4">
      <div class="table-container col-span-3 space-y-6 p-3 pt-5 lg:col-start-2">
        <table class="table table-hover">
          <thead>
            <tr>
              <th>Preset Name</th>
              <th>IP Address</th>
              <th>Subnet Mask</th>
              <th>Options</th>
              <th>Operation</th>
            </tr>
          </thead>
          <tbody>
            {#each form.data.network as iface, i}
              <tr>
                <td>
                  <label class="label">
                    <input
                      required
                      class="input {form.duplicate.network.includes(iface.name)
                        ? 'input-error'
                        : ''}"
                      type="text"
                      bind:value={iface.name}
                      on:change={() => {
                        handleDuplicate('network');
                      }} />
                  </label>
                </td>
                <td>
                  <label class="label">
                    <input
                      required
                      class="input {form.duplicate.address.includes(iface.address)
                        ? 'input-error'
                        : ''}"
                      type="text"
                      minlength="7"
                      maxlength="15"
                      pattern="^((\d|[1-9]\d|1\d\d|2[0-4]\d|25[0-5])\.){'{'}3}(\d|[1-9]\d|1\d\d|2[0-4]\d|25[0-5])$"
                      placeholder="x.x.x.x"
                      title="IP Address x.x.x.x"
                      bind:value={iface.address}
                      on:change={() => {
                        handleDuplicate('address');
                      }} />
                  </label>
                </td>
                <td>
                  <label class="label">
                    <input
                      required
                      class="input"
                      type="text"
                      minlength="7"
                      maxlength="15"
                      pattern="^(((255\.){'{'}3}(25[245]|24[08]|224|192|128))|((255\.){'{'}2}(25[245]|24[08]|224|192|128)\.0)|(255\.(25[245]|24[08]|224|192|128)\.0\.0)|((25[245]|24[08]|224|192|128|0)\.0\.0\.0))$"
                      placeholder="x.x.x.x"
                      title="Subnet Mask x.x.x.x"
                      bind:value={iface.netmask} />
                  </label>
                </td>
                <td>
                  <label class="label">
                    <input
                      class="checkbox"
                      type="checkbox"
                      bind:checked={iface.extraOptions.autoneg_off} />
                    <span>Autoneg Off (100 Mbit/sec)</span>
                  </label>
                </td>
                <td>
                  <button
                    type="button"
                    class="variant-filled btn mb-1 w-7 rounded p-1"
                    on:click|preventDefault={() => handleUp('network', i)}>
                    <i class="fa-solid fa-arrow-up"></i>
                  </button>
                  <button
                    type="button"
                    class="variant-filled btn mb-1 w-7 rounded p-1"
                    on:click|preventDefault={() => handleDown('network', i)}>
                    <i class="fa-solid fa-arrow-down"></i>
                  </button>
                  <button
                    type="button"
                    class="variant-filled btn mb-1 w-7 rounded p-1"
                    on:click|preventDefault={() => handleCopy('network', i)}>
                    <i class="fa-solid fa-copy"></i>
                  </button>
                  <button
                    type="button"
                    class="variant-filled btn mb-1 w-7 rounded p-1"
                    on:click|preventDefault={() => handleDelete('network', i)}>
                    <i class="fa-solid fa-times"></i>
                  </button>
                </td>
              </tr>
            {:else}
              <tr>
                <td colspan="5">No network configurations yet.</td>
              </tr>
            {/each}
          </tbody>
        </table>
        <div class="mt-5">
          <button
            type="button"
            class="variant-filled-secondary btn p-5 text-white"
            on:click|preventDefault={() => handleAdd('network')}
            tip-title="Add new network configuration">
            <i class="fa-solid fa-plus"></i>
          </button>
        </div>
      </div>
    </div>

    <legend class="legend py-5">
      <span class={formLegendClass}>Environment Variable</span>
    </legend>
    <hr />
    <div class="grid grid-cols-3 lg:grid-cols-4">
      <div class="table-container col-span-3 space-y-6 p-3 pt-5 lg:col-start-2">
        <table class="table table-hover">
          <thead>
            <tr>
              <th>#</th>
              <th>Identifier</th>
              <th>Value</th>
              <th>Operation</th>
            </tr>
          </thead>
          <tbody>
            {#each form.data.env as env, i}
              <tr>
                <td>{i + 1}</td>
                <td>
                  <label class="label">
                    <input
                      required
                      class="input {form.duplicate.env.includes(env[0]) ? 'input-error' : ''}"
                      type="text"
                      bind:value={env[0]}
                      on:change={() => {
                        handleDuplicate('env');
                      }} />
                  </label>
                </td>
                <td>
                  <label class="label">
                    <input required class="input" type="text" bind:value={env[1]} />
                  </label>
                </td>
                <td>
                  <button
                    type="button"
                    class="variant-filled btn mb-1 w-7 rounded p-1"
                    on:click|preventDefault={() => handleUp('env', i)}>
                    <i class="fa-solid fa-arrow-up"></i>
                  </button>
                  <button
                    type="button"
                    class="variant-filled btn mb-1 w-7 rounded p-1"
                    on:click|preventDefault={() => handleDown('env', i)}>
                    <i class="fa-solid fa-arrow-down"></i>
                  </button>
                  <button
                    type="button"
                    class="variant-filled btn mb-1 w-7 rounded p-1"
                    on:click|preventDefault={() => handleCopy('env', i)}>
                    <i class="fa-solid fa-copy"></i>
                  </button>
                  <button
                    type="button"
                    class="variant-filled btn mb-1 w-7 rounded p-1"
                    on:click|preventDefault={() => handleDelete('env', i)}>
                    <i class="fa-solid fa-times"></i>
                  </button>
                </td>
              </tr>
            {:else}
              <tr>
                <td colspan="4">No environment variables yet.</td>
              </tr>
            {/each}
          </tbody>
        </table>
        <div class="mt-5">
          <button
            type="button"
            class="variant-filled-secondary btn p-5 text-white"
            on:click|preventDefault={() => handleAdd('env')}
            tip-title="Add new environment variable">
            <i class="fa-solid fa-plus"></i>
          </button>
        </div>
      </div>
    </div>

    <div class="mt-10 grid grid-cols-3 lg:grid-cols-4">
      <div class="col-span-2 col-start-2">
        <button type="submit" class="variant-filled-primary btn">Submit</button>
      </div>
    </div>
  </form>
</ConfigLayout>
