<script>
  import { beforeNavigate, goto, invalidateAll } from '$app/navigation';
  import { getToastStore } from '@skeletonlabs/skeleton';

  import agvConfig from '$lib/shared/services/config/agv';
  import ConfigLayout from 'components/ConfigLayout.svelte';
  import dimensionEditorF from './dimension/editor';
  import Select from 'components/Select.svelte';
  import SkillsetImg from 'components/SkillsetImg.svelte';
  import taskTriggerEditorF from './task-trigger/editor';

  export let data;

  const toastStore = getToastStore();

  const defaultAppOptions = [
    {
      name: '---------',
      value: ''
    },
    {
      name: 'Task Runner',
      value: 'task-runner'
    }
  ];

  const commonFields = ['agv_name', 'executor_mode'];
  const standaloneFields = [
    'agv_home',
    'custom_init',
    'station_init',
    'station_init_stations',
    'pre_init',
    'default_init',
    'default_init_timeout',
    'default_paused',
    'default_app',
    'default_app_timeout',
    'min_battery_level',
    'dimension',
    'task_triggers',
    'allowed_motions'
  ];
  const dfleetFields = ['fms_endpoint'];

  let dirty = false;
  let commonForm;
  let dfleetForm;
  let standaloneForm;

  let dimensionEditor;
  let taskTriggerEditor;
  let agv = data.agv;

  let initChoices = data.initChoices;
  let preInitChoices = data.preInitChoices;
  let defaultInitChoices = [];
  let stationInitChoices = data.stationInitChoices;
  let stationChoices = data.stationChoices;

  // sanitize data
  if (agv.custom_init && Array.isArray(agv.custom_init)) {
    let choices = initChoices.flatMap((o) => o.options.map((c) => c[0]));
    agv.custom_init = agv.custom_init.filter((c) => choices.includes(c));
  }
  if (agv.pre_init && Array.isArray(agv.pre_init)) {
    let choices = preInitChoices.flatMap((o) => o.options.map((c) => c[0]));
    agv.pre_init = agv.pre_init.filter((c) => choices.includes(c));
  }

  updateDefaultInitChoices();

  function updateDefaultInitChoices() {
    let newDefaultInitChoices = [
      {
        name: '_',
        options: [['', '-----------']]
      },
      {
        name: 'Built-in',
        options: []
      },
      {
        name: 'Task Template',
        options: []
      },
      {
        name: 'Task Template (Top-Level)',
        options: []
      }
    ];
    for (let option of initChoices[0].options) {
      if (agv.custom_init.includes(option[0])) {
        newDefaultInitChoices[1].options.push(option);
      }
    }
    for (let option of initChoices[1].options) {
      if (agv.custom_init.includes(option[0])) {
        newDefaultInitChoices[2].options.push(option);
      }
    }
    for (let option of initChoices[2].options) {
      if (agv.custom_init.includes(option[0])) {
        newDefaultInitChoices[3].options.push(option);
      }
    }
    defaultInitChoices = newDefaultInitChoices;
    if (!agv.custom_init.includes(agv.default_init)) {
      agv.default_init = '';
    }
  }

  function editorDirty() {
    if (!dirty) {
      dirty = true;
    }
  }

  function dimensionEditorReady(e) {
    dimensionEditor = e.target.editor;
  }

  function taskTriggerEditorReady(e) {
    taskTriggerEditor = e.target.editor;
  }

  function agvFormSubmit() {
    if (!commonForm.checkValidity()) {
      commonForm.reportValidity();
      return;
    }

    if (Number.parseInt(commonForm.executor_mode.value) === 1) {
      // Standalone mode.
      if (!standaloneForm.checkValidity()) {
        standaloneForm.reportValidity();
        return;
      }
    } else if (Number.parseInt(commonForm.executor_mode.value) === 2) {
      // DFleet mode.
      if (!dfleetForm.checkValidity()) {
        dfleetForm.reportValidity();
        return;
      }
    }
    save();
  }

  function save() {
    if (Number.parseInt(commonForm.executor_mode.value) === 1) {
      // Standalone mode
      dimensionEditor.stage();
      taskTriggerEditor.stage();
      let payload = {};
      for (const key of commonFields) {
        payload[key] = agv[key];
      }
      for (const key of standaloneFields) {
        payload[key] = agv[key];
      }

      agvConfig
        .update(payload)
        .then(() => {
          toastStore.trigger({
            message: agvConfig.successUpdateMsg(agv.agv_name),
            timeout: 3000,
            hoverable: true
          });
          // Prevent using preloaded data
          dirty = false;
          goto('/config/agv').then(() => invalidateAll());
        })
        .catch((e) => {
          toastStore.trigger({
            message: e,
            timeout: 3000,
            hoverable: true
          });
        });
    } else if (Number.parseInt(commonForm.executor_mode.value) === 2) {
      // Dfleet mode
      let payload = {};
      for (const key of commonFields) {
        payload[key] = agv[key];
      }
      for (const key of dfleetFields) {
        payload[key] = agv[key];
      }

      agvConfig
        .update(payload)
        .then((d) => {
          toastStore.trigger({
            message: agvConfig.successUpdateDfleetModeMsg(d.url, d.is_active),
            timeout: 6000,
            hoverable: true
          });
          // Prevent using preloaded data
          dirty = false;
          goto('/config/agv').then(() => invalidateAll());
        })
        .catch((e) => {
          toastStore.trigger({
            message: e,
            timeout: 3000,
            hoverable: true
          });
        });
    }
  }

  function triggerDirty() {
    if (!dirty) {
      dirty = true;
    }
  }

  beforeNavigate((e) => {
    if (dirty) {
      if (!confirm('There are unsaved changes on this page.')) {
        e.cancel();
      }
    }
  });

  const helpText = 'text-slate-400 italic';
</script>

<ConfigLayout title="AGV">
  <div class="grid grid-cols-12">
    <div class="col-span-9">
      <form bind:this={commonForm} action="" on:submit|preventDefault class="space-y-4 pl-3">
        <div class="grid grid-cols-9">
          <span class="col-span-2 font-medium">AGV Name</span>
          <input
            class="input col-span-7"
            type="text"
            name="agv_name"
            bind:value={agv.agv_name}
            on:change={triggerDirty}
            required />
        </div>

        <div class="grid grid-cols-9">
          <span class="col-span-2 font-medium">UUID</span>
          <div class="input-group input-group-divider col-span-7 grid-cols-[1fr_auto]">
            <div class="px-4 py-2 pl-5 opacity-70">{agv.agv_uuid}</div>
          </div>
        </div>

        <div class="grid grid-cols-9">
          <span class="col-span-2 font-medium">Executor Mode</span>
          <div class="col-span-7">
            <label class="flex items-center space-x-2">
              <input
                class="radio"
                type="radio"
                bind:group={agv.executor_mode}
                name="executor_mode"
                value="1"
                required />
              <p>Standalone</p>
            </label>
            <label class="flex items-center space-x-2">
              <input
                class="radio"
                type="radio"
                bind:group={agv.executor_mode}
                name="executor_mode"
                value="2"
                required />
              <p>DFleet</p>
            </label>
            <p class={helpText}>Set executor mode</p>
          </div>
        </div>
      </form>

      <form bind:this={dfleetForm} action="" on:submit|preventDefault>
        <div class="grid grid-cols-9 pl-3 pt-3" class:hidden={agv.executor_mode === '1'}>
          <span class="col-span-2 font-medium">DFleet Endpoint</span>
          <div class="col-span-7">
            <input
              class="input"
              type="text"
              name="fms_endpoint"
              bind:value={agv.fms_endpoint}
              on:change={triggerDirty}
              required />
            <p class={helpText}>Specify DFleet endpoint URL</p>
          </div>
        </div>
      </form>
    </div>

    <div class="pl-7">
      <SkillsetImg
        src={agv.skillset_img}
        skillset={agv.skillset}
        placement="left"
        offset={50} />
    </div>

    <form
      bind:this={standaloneForm}
      action=""
      on:submit|preventDefault
      class:hidden={agv.executor_mode !== '1' && dimensionEditor}
      class="col-span-full grid grid-cols-12 space-y-4 pt-3">
      <div class="col-span-9 grid grid-cols-9 pl-3">
        <span class="col-span-2 font-medium">AGV Home</span>
        <div class="col-span-7">
          <select
            class="select rounded-token"
            bind:value={agv.agv_home}
            name="agv_home"
            on:change={triggerDirty}>
            {#each JSON.parse(agv.station_list) as station}
              <option>{station}</option>
            {/each}
          </select>
          <p class={helpText}>Specify AGV's home station</p>
        </div>
      </div>

      <div class="col-span-full">
        <h2 class="text-2xl tracking-widest">Dimension Profiles</h2>
        <hr class="mb-4" />
        <div
          class="editor"
          on:ready={dimensionEditorReady}
          on:dirty={editorDirty}
          use:dimensionEditorF={agv}>
        </div>
      </div>

      <div class="col-span-full">
        <h2 class="text-2xl tracking-widest">Executor Settings</h2>
        <hr class="mb-4" />
        <div class="grid grid-cols-12 p-3">
          <span class="col-span-2 font-medium">Allowed Motions</span>
          <div class="col-span-7 space-y-2">
            {#each data.allowedMotionsChoices as am}
              <label class="flex items-center space-x-2">
                <input
                  class="checkbox"
                  type="checkbox"
                  value={am.value}
                  name="allowed_motions"
                  bind:group={agv.allowed_motions}
                  on:change={triggerDirty} />
                <p>{am.name}</p>
              </label>
            {/each}
          </div>
        </div>
      </div>

      <div class="col-span-full p-3">
        <div
          class="editor"
          on:ready={taskTriggerEditorReady}
          on:dirty={editorDirty}
          use:taskTriggerEditorF={agv}>
        </div>
      </div>

      <div class="col-span-full grid grid-cols-12 p-3">
        <span class="col-span-2 font-medium">Initialization Task Template</span>
        <div class="col-span-7">
          <Select
            name="custom_init"
            multiple={true}
            startEmpty={false}
            enableFilter={true}
            buttonClass="px-7 variant-form"
            class="w-full max-w-lg"
            options={initChoices}
            bind:value={agv.custom_init}
            on:change={updateDefaultInitChoices}
            required />
          <p class={helpText}>Task template for initialization of task runner.</p>
        </div>
      </div>

      <div class="col-span-full grid grid-cols-12 p-3">
        <span class="col-span-2 font-medium">Initialization at a Station</span>
        <div class="col-span-7 space-y-2">
          {#each stationInitChoices as choice}
            <label class="flex items-center space-x-2">
              <input
                class="radio"
                type="radio"
                bind:group={agv.station_init}
                name="station_init"
                value={choice[0]}
                required />
              <p>{choice[1]}</p>
            </label>
          {/each}
        </div>
        {#if agv.station_init === 2}
          <div class="col-span-7 col-start-3 pt-3">
            <Select
              name="station_init_stations"
              multiple={true}
              startEmpty={false}
              enableFilter={false}
              buttonClass="px-7 variant-form"
              class="w-full max-w-lg"
              options={stationChoices}
              value={agv.station_init_stations}
              required />
            <p class={helpText}>Whether to allow initialization at a station.</p>
          </div>
        {/if}
      </div>

      <div class="col-span-full grid grid-cols-12 p-3">
        <span class="col-span-2 font-medium">Pre Initialization Task Template</span>
        <div class="col-span-7">
          <Select
            name="pre_init"
            multiple={true}
            startEmpty={false}
            enableFilter={true}
            buttonClass="px-7 variant-form"
            class="w-full max-w-lg"
            options={preInitChoices}
            value={agv.pre_init} />
          <p class={helpText}>
            Task template for pre-initialization of task runner. Leaving it empty will disable
            the function.
          </p>
        </div>
      </div>

      <div class="col-span-full grid grid-cols-12 p-3">
        <span class="col-span-2 font-medium">Default Initialization Task Template</span>
        <div class="col-span-7">
          <Select
            name="default_init"
            startEmpty={true}
            enableFilter={true}
            buttonClass="px-7 variant-form"
            class="w-full max-w-lg"
            options={defaultInitChoices}
            bind:value={agv.default_init} />
          <p class={helpText}>
            Task template for default initialization of task runner. Leaving it empty will
            disable the function.
          </p>
        </div>
      </div>

      <div class="col-span-full grid grid-cols-12 p-3">
        <span class="col-span-2 font-medium">Default Initialization Timeout</span>
        <div class="col-span-7">
          <input
            class="input max-w-lg"
            type="number"
            name="default_init_timeout"
            min="1"
            max="9999"
            bind:value={agv.default_init_timeout}
            on:change={triggerDirty}
            required />
          <p class={helpText}>
            Perform default initialization if there is no user input for more than this
            duration (in seconds).
          </p>
        </div>
      </div>

      <div class="col-span-full grid grid-cols-12 p-3">
        <label class="col-span-7 col-start-3 flex items-center space-x-2">
          <input
            name="default_paused"
            class="checkbox col-span-3"
            type="checkbox"
            bind:checked={agv.default_paused}
            on:change={triggerDirty} />
          <p class="font-medium">Default to starting task runner in paused state.</p>
        </label>
      </div>

      <div class="col-span-full grid grid-cols-12 p-3">
        <span class="col-span-2 font-medium">Default App</span>
        <div class="col-span-7">
          <select
            class="select max-w-lg rounded-token"
            name="default_app"
            bind:value={agv.default_app}
            on:change={triggerDirty}>
            {#each defaultAppOptions as app}
              <option value={app.value}>{app.name}</option>
            {/each}
            required
          </select>
          <p class={helpText}>Specify AGV's home station</p>
        </div>
      </div>

      <div class="col-span-full grid grid-cols-12 p-3">
        <span class="col-span-2 font-medium">Default App Timeout</span>
        <div class="col-span-7">
          <input
            class="input col-span-3 max-w-lg"
            type="number"
            min="1"
            max="9999"
            name="default_app_timeout"
            bind:value={agv.default_app_timeout}
            on:change={triggerDirty}
            required />
          <p class={helpText}>
            Start default app if there is no user input for more than this duration (in
            seconds).
          </p>
        </div>
      </div>

      <div class="col-span-full grid grid-cols-12 p-3">
        <span class="col-span-2 font-medium">Minimum Battery Level</span>
        <div class="col-span-7">
          <input
            class="input col-span-3 max-w-lg"
            name="min_battery_level"
            type="number"
            min="0"
            max="100"
            bind:value={agv.min_battery_level}
            on:change={triggerDirty}
            required />
          <p class={helpText}>Minimum AGV battery level (%) for executing a new task.</p>
        </div>
      </div>
    </form>

    <div class="col-span-3 col-start-3 mt-6 p-3">
      <button type="submit" class="variant-filled-primary btn w-32" on:click={agvFormSubmit}>
        Submit
      </button>
    </div>
  </div>
</ConfigLayout>

<style>
  @import '$lib/styles/viz.css';
  @import '$lib/styles/map-viz.css';
</style>
