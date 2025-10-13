<script>
  import { goto } from '$app/navigation';
  import { startCase } from 'lodash-es';
  import { getModalStore } from '@skeletonlabs/skeleton';
  import { onMount, getContext } from 'svelte';
  import { ProgressRadial } from '@skeletonlabs/skeleton';
  import cashDom from 'cash-dom';

  import AppLayout from 'components/AppLayout.svelte';
  import { formatMinMax } from '$lib/utils';
  import { alertModal } from '$lib/modal-service.js';
  import markerF from 'task-template-editor/toolbar/marker';
  import KeyboardSpacer from 'components/keyboard/KeyboardSpacer.svelte';

  const modalStore = getModalStore();
  const mainController = getContext('mainController');
  const skillTest = mainController.moduleService.getSub('skill-test');

  let form;
  let skillOptions = [];
  let paramDescriptions = [];
  let stations = [];
  let registers = [];
  let variables = [];

  let data = {
    skill: null,
    params: {}
  };
  let ready = false;
  let skillRunning = false;
  let skillText = '';

  function skillTestCallback(_data) {
    if (_data.command === 'list_all') {
      if (
        Array.isArray(_data.skills) &&
        Array.isArray(_data.stations) &&
        Array.isArray(_data.registers) &&
        Array.isArray(_data.variables)
      ) {
        skillOptions = _data.skills;
        stations = _data.stations;
        registers = _data.registers;
        variables = _data.variables;

        data.skill = skillOptions[0];
        ready = true;
        skillChanged();
      }
    } else if (_data.command === 'status') {
      skillRunning = _data.status === 'running';
      skillText = _data.text;
    } else if (_data.command === 'error') {
      modalStore.trigger(alertModal('Action Error', _data.error));
    } else if (_data.command === 'outcome') {
      modalStore.trigger(alertModal('Action Outcome', _data.outcome));
    }
  }

  function getDefault(param) {
    if (param.type === 'bool') {
      return false;
    } else if (param.type === 'int' || param.type === 'double') {
      if (param.min && param.min > 0) {
        return param.min;
      } else if (param.max && param.max < 0) {
        return param.max;
      } else {
        return 0;
      }
    } else if (param.type === 'str') {
      return '';
    } else if (param.type === 'Register') {
      return registers && registers.length ? registers[0] : null;
    } else if (param.type === 'Station') {
      return stations && stations.length ? stations[0] : null;
    } else if (param.type.startsWith('v')) {
      if (!variables || variables.length <= 0) {
        return null;
      }
      let variableList = variables.filter((o) => o.vtype === param.type);
      return variableList.length ? variableList[0].name : null;
    }
  }

  function skillChanged() {
    paramDescriptions = [];
    data.params = {};
    markerModalKey = undefined;
    for (let param of data.skill.params) {
      param = JSON.parse(param);
      param.key = param.name;
      if (param.version) {
        param.key += ':' + param.version;
      }
      paramDescriptions.push(param);
      data.params[param.key] = param.default ? param.default : getDefault(param);
    }
  }

  function startSkill() {
    if (skillRunning) {
      return;
    }
    if (!form.checkValidity()) {
      form.reportValidity();
      return;
    }
    skillTest.publish({
      command: 'start_skill',
      skill_id: data.skill.id,
      skill_params: data.params
    });
  }

  function stopSkill() {
    if (!skillRunning) {
      return;
    }
    skillTest.publish({
      command: 'stop_skill'
    });
  }

  function shutdown() {
    const modal = {
      type: 'confirm',
      title: 'Confirm stopping module <br/> "Action Test" ?',
      response: (r) => {
        if (r) {
          skillTest.stopModule();
          goto('/apps', { replaceState: true });
        }
      }
    };
    modalStore.trigger(modal);
  }

  let markerModalVisible = false;
  let markerModal;
  let markerModalKey;
  function markerLoad(parentDom) {
    let parent = cashDom(parentDom);
    markerModal = markerF(parent, true);
    markerModal.on('apply', (e, value) => {
      if (!markerModalKey) {
        return;
      }
      data.params[markerModalKey] = value;
    });
    markerModal.on('visibility', () => {
      markerModalVisible = markerModal.visible();
    });
  }

  function showMarkerModal(key) {
    if (!markerModal) {
      return;
    }
    markerModalKey = key;
    markerModal.show(data.params[key]);
  }

  onMount(() => {
    skillTest.subscribe(skillTestCallback);
    // allow time for subscribe to happen first.
    setTimeout(function () {
      skillTest.publish({
        command: 'list_all'
      });
      skillTest.publish({
        command: 'status'
      });
    }, 100);

    return () => {
      skillTest.unsubscribe(skillTestCallback);
    };
  });
</script>

<AppLayout
  title="Action Test"
  moduleID="skill-test"
  keyboardSpacer={!markerModalVisible}
  on:close={shutdown}>
  <div class="p-4 pt-8">
    <div class="card mx-8 grid grid-cols-1 px-4 pt-2 shadow-lg md:grid-cols-[1fr_8fr]">
      <span class="my-auto pr-10 font-semibold md:text-right">Select action:</span>
      <section class="grid max-w-5xl grid-cols-1 gap-2 py-4 md:grid-cols-[1fr_auto]">
        <select class="select rounded-token" bind:value={data.skill} on:change={skillChanged}>
          {#each skillOptions as skill}
            <option value={skill}>
              {skill.name}
            </option>
          {/each}
        </select>
        <div class="flex space-x-2">
          {#if ready}
            <button
              type="button"
              class="variant-filled-success btn"
              on:click={startSkill}
              disabled={skillRunning}>Start</button>
            <button
              type="button"
              class="variant-filled-error btn"
              on:click={stopSkill}
              disabled={!skillRunning}>Stop</button>
          {:else}
            <ProgressRadial width="w-10 m-auto" />
          {/if}
        </div>
        <div>
          Now:
          {#if ready && skillRunning && skillText}
            <span>{skillText}</span>
          {:else}
            <span>-</span>
          {/if}
        </div>
      </section>
    </div>
    <form class="space-y-4 px-4 pt-10" bind:this={form} on:submit|preventDefault novalidate>
      {#each paramDescriptions as d}
        <div class="grid grid-cols-1 md:grid-cols-[1fr_7fr_1fr]">
          <span class="mb-1 pr-10 font-semibold md:text-right">{startCase(d.name)}</span>
          <div>
            {#if d.type === 'str'}
              <div class="input-group input-group-divider grid-cols-[1fr_auto]">
                <input class="keyboard input" type="text" bind:value={data.params[d.key]} />
                {#if d.name === 'marker_type'}
                  <button
                    type="button"
                    on:click={showMarkerModal(d.key)}
                    tip-title="Marker Type Configuration">
                    <i class="fa fa-info-circle"></i>
                  </button>
                {/if}
              </div>
            {:else if ['int', 'double'].indexOf(d.type) >= 0}
              <input
                type="number"
                class="keyboard input"
                bind:value={data.params[d.key]}
                min={d.min}
                max={d.max}
                step={d.type === 'int' ? 1 : 'any'}
                required />
              <span class="text-sm italic text-red-700">{formatMinMax(d)}</span>
            {:else if d.type === 'bool'}
              <div class="toggle toggle-balanced">
                <input class="checkbox" type="checkbox" bind:checked={data.params[d.key]} />
                <div class="track">
                  <div class="handle"></div>
                </div>
              </div>
            {:else if d.type === 'Station'}
              <select class="select rounded-token" bind:value={data.params[d.key]} required>
                {#each stations as st}
                  <option value={st}>
                    {st}
                  </option>
                {/each}
              </select>
            {:else if d.type === 'Register'}
              <select class="select rounded-token" bind:value={data.params[d.key]} required>
                {#each registers as reg}
                  <option value={reg}>
                    {reg}
                  </option>
                {/each}
              </select>
            {:else if d.type.startsWith('v')}
              <select class="select rounded-token" bind:value={data.params[d.key]} required>
                {#each variables.filter((v) => v.vtype === d.type) as v}
                  <option value={v.name}>
                    {v.name}
                  </option>
                {/each}
              </select>
            {/if}
          </div>
          <p class="italic text-gray-700 md:col-start-2">{d.description}</p>
        </div>
      {/each}
    </form>
    <div class="fixed inset-0 flex flex-col" class:hidden={!markerModalVisible}>
      <div class="relative flex-grow" use:markerLoad></div>
      <KeyboardSpacer class="backdrop-blur-sm backdrop-contrast-50" />
    </div>
  </div>
</AppLayout>

<style>
  input:invalid {
    border: 1px solid red;
  }
</style>
