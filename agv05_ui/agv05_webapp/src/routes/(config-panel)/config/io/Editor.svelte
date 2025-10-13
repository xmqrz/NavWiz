<script>
  import { getModalStore, TabGroup, Tab } from '@skeletonlabs/skeleton';

  const modalStore = getModalStore();

  const buttonClass = 'variant-ghost-surface rounded-lg btn block text-wrap';

  export let PORT;
  export let PIN;
  export let outputName;
  export let inputName;
  export let update;

  let tab = 0;

  function prompt(port, pin, isOutput) {
    var initial;
    var type;

    if (isOutput) {
      initial = outputName[port][pin];
      type = 'output';
    } else {
      initial = inputName[port][pin];
      type = 'input';
    }

    modalStore.trigger({
      type: 'prompt',
      title: `Edit ${type} pin ${pin}`,
      body: `Enter the name for ${type} pin ${pin}`,
      buttonTextSubmit: 'Update',
      value: initial,
      response: (name) => {
        if (name === false) {
          return;
        }
        name = name || '';
        if (name !== initial) {
          update(port, pin, isOutput, name);
        }
      }
    });
  }
</script>

<TabGroup>
  {#each { length: PORT } as _, i}
    {@const port = i + 1}
    <Tab bind:group={tab} name="port{name}" value={i}>Port {port}</Tab>
  {/each}
  <svelte:fragment slot="panel">
    {#each { length: PORT } as _, i}
      <div class="space-y-6 pt-3 sm:px-6" class:hidden={tab !== i}>
        <div class="space-y-3">
          <span class="text-lg font-semibold">Outputs</span>
          <div class="grid grid-cols-2 sm:grid-cols-4 md:grid-cols-8">
            {#each { length: PIN } as _, j}
              {@const pin = 15 - j}
              <button type="button" class={buttonClass} on:click={() => prompt(i, pin, true)}>
                O{pin}<br /><small>{outputName[i][pin]}</small>
              </button>
            {/each}
          </div>
        </div>
        <div class="space-y-3">
          <span class="text-lg font-semibold">Inputs</span>
          <div class="grid grid-cols-2 sm:grid-cols-4 md:grid-cols-8">
            {#each { length: PIN } as _, j}
              {@const pin = 15 - j}
              <button type="button" class={buttonClass} on:click={() => prompt(i, pin, false)}>
                I{pin}<br /><small>{inputName[i][pin]}</small>
              </button>
            {/each}
          </div>
        </div>
      </div>
    {/each}
  </svelte:fragment>
</TabGroup>
