<script>
  import { getModalStore } from '@skeletonlabs/skeleton';
  import { startCase } from 'lodash-es';

  import KeyboardBackdrop from 'components/keyboard/KeyboardBackdrop.svelte';
  import { formatMinMax } from '$lib/utils';

  // Props
  export let parent;

  const cBase = 'card p-4 w-modal shadow-2xl space-y-4 rounded-3xl';
  const cHeader = 'text-2xl font-bold';
  const cForm =
    'grow overflow-auto border border-surface-500 p-4 space-y-4 rounded-container-token';
  const modalStore = getModalStore();

  const formData = $modalStore[0].meta.data.params;

  function onFormSubmit() {
    if ($modalStore[0].response) {
      $modalStore[0].response(formData);
    }
    modalStore.close();
  }
</script>

{#if $modalStore[0] && $modalStore[0].meta}
  <KeyboardBackdrop>
    <div class="{cBase} m-auto flex flex-col" style="max-height: 90vh;">
      <header class={cHeader}>{$modalStore[0].title ?? 'Title undefined'}</header>
      <form name="taskParamsForm" class={cForm}>
        {#if $modalStore[0].meta && $modalStore[0].meta.template && $modalStore[0].meta.template.params}
          {#each $modalStore[0].meta.template.params as d (d.name)}
            <label class="label">
              <span>{startCase(d.name)}</span>
              <div>
                {#if d.type === 'str'}
                  <input class="keyboard input" type="text" bind:value={formData[d.name]} />
                {:else if ['int', 'double'].indexOf(d.type) >= 0}
                  <input
                    class="keyboard input"
                    type="number"
                    bind:value={formData[d.name]}
                    min={d.min}
                    max={d.max}
                    step={d.type === 'int' ? 1 : 'any'}
                    title={formatMinMax(d)}
                    required />
                {:else if d.type === 'bool'}
                  <div class="toggle toggle-balanced">
                    <input class="checkbox" type="checkbox" bind:checked={formData[d.name]} />
                    <div class="track">
                      <div class="handle"></div>
                    </div>
                  </div>
                {:else if d.type === 'Station'}
                  <select class="select rounded-token" bind:value={formData[d.name]} required>
                    {#each $modalStore[0].meta.stations as st}
                      <option value={st}>
                        {st}
                      </option>
                    {/each}
                  </select>
                {:else if d.type === 'Register'}
                  <select class="select rounded-token" bind:value={formData[d.name]} required>
                    {#each $modalStore[0].meta.registers as reg}
                      <option value={reg}>
                        {reg}
                      </option>
                    {/each}
                  </select>
                {:else if d.type.startsWith('v')}
                  <select class="select rounded-token" bind:value={formData[d.name]} required>
                    {#each $modalStore[0].meta.variables.filter((v) => v.vtype === d.type) as v}
                      <option value={v.name}>
                        {v.name}
                      </option>
                    {/each}
                  </select>
                {/if}
              </div>
            </label>
          {/each}
        {/if}
      </form>
      <footer class="modal-footer mt-2 {parent.regionFooter}">
        <button type="button" class="btn {parent.buttonNeutral}" on:click={parent.onClose}>
          {parent.buttonTextCancel}
        </button>
        <button type="button" class="btn {parent.buttonPositive}" on:click={onFormSubmit}>
          OK
        </button>
      </footer>
    </div>
  </KeyboardBackdrop>
{/if}

<style>
  input:invalid {
    border: 1px solid red;
  }
</style>
