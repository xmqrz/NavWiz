<script>
  import { ioChannel } from 'stores/sock';
  import { onMount } from 'svelte';
  import { getModalStore } from '@skeletonlabs/skeleton';

  import TaskRunnerModalLayout from './TaskRunnerModalLayout.svelte';

  // Props
  export let parent;
  export let activeModule;

  const modalStore = getModalStore();
  let io = new Io();

  function Io() {
    this._inputs = [0, 0, 0, 0, 0, 0, 0, 0];
    this._outputs = [0, 0, 0, 0, 0, 0, 0, 0];
    this._ioName = null;

    this.getInput = (port, pin) => this._inputs[port - 1] & (1 << pin);
    this.getOutput = (port, pin) => this._outputs[port - 1] & (1 << pin);
  }
  Io.prototype.getOutputName = function (port, pin) {
    if (!this._ioName || !this._ioName.output_name) {
      return '';
    }
    return this._ioName.output_name[port - 1][pin];
  };
  Io.prototype.getInputName = function (port, pin) {
    if (!this._ioName || !this._ioName.input_name) {
      return '';
    }
    return this._ioName.input_name[port - 1][pin];
  };
  function updateIo(data) {
    if (!data) {
      return;
    }
    if (!io._ioName && data.io_name) {
      io._ioName = data.io_name;
    }
    if (Array.isArray(data.inputs) && data.inputs.length === 8) {
      io._inputs = data.inputs;
    }
    if (Array.isArray(data.outputs) && data.outputs.length === 8) {
      io._outputs = data.outputs;
    }
  }

  // callback
  function ioCallback(data) {
    updateIo(data);
  }

  onMount(() => {
    io.port = 1;
    ioChannel.subscribe(ioCallback);
    return () => {
      ioChannel.unsubscribe(ioCallback);
    };
  });

  const gridStretch = 'grid grid-cols-8 place-content-stretch';
</script>

{#if $modalStore[0]}
  <TaskRunnerModalLayout {activeModule}>
    <div class="panel-modal card flex min-h-[350px] min-w-[900px] flex-col rounded-3xl">
      <div class="variant-filled-secondary rounded-t-2xl p-4">
        <span class="text-center font-bold">I/O (View Only)</span>
        <div class="float-right">
          <button type="button" class="btn btn-sm" on:click={parent.onClose}>
            <i class="fa-solid fa-xmark"></i>
          </button>
        </div>
      </div>
      <div class="flex grow flex-col justify-between space-y-2 overflow-auto p-4">
        <div class={gridStretch}>
          {#each [1, 2, 3, 4, 5, 6, 7, 8] as i}
            <button
              type="button"
              class="p-4 hover:bg-slate-300 dark:text-slate-500 {io.port === i
                ? 'bg-slate-300'
                : 'bg-white'}"
              on:click={() => (io.port = i)}>Port {i}</button>
          {/each}
        </div>
        <div class="flex-grow">
          <div class={gridStretch}>
            {#each [15, 14, 13, 12, 11, 10, 9, 8] as pin}
              <button
                type="button"
                class="p-2 {io.getOutput(io.port, pin)
                  ? 'variant-filled-success'
                  : 'variant-filled-error'}"
                style="vertical-align: top; white-space: normal; overflow: hidden; line-height: 20px">
                {'O' + pin}<br /><small>{io.getOutputName(io.port, pin)}</small>
              </button>
            {/each}
          </div>
          <div class={gridStretch}>
            {#each [7, 6, 5, 4, 3, 2, 1, 0] as pin}
              <button
                type="button"
                class="p-2 {io.getOutput(io.port, pin)
                  ? 'variant-filled-success'
                  : 'variant-filled-error'}"
                style="vertical-align: top; white-space: normal; overflow: hidden; line-height: 20px">
                {'O' + pin}<br /><small>{io.getOutputName(io.port, pin)}</small>
              </button>
            {/each}
          </div>
        </div>
        <div class="flex-grow">
          <div class={gridStretch}>
            {#each [15, 14, 13, 12, 11, 10, 9, 8] as pin}
              <button
                type="button"
                class="p-2 {io.getInput(io.port, pin)
                  ? 'variant-filled-success'
                  : 'variant-filled-error'}"
                style="vertical-align: top; white-space: normal; overflow: hidden; line-height: 20px">
                {'I' + pin}<br /><small>{io.getInputName(io.port, pin)}</small>
              </button>
            {/each}
          </div>
          <div class={gridStretch}>
            {#each [7, 6, 5, 4, 3, 2, 1, 0] as pin}
              <button
                type="button"
                class="p-2 {io.getInput(io.port, pin)
                  ? 'variant-filled-success'
                  : 'variant-filled-error'}"
                style="vertical-align: top; white-space: normal; overflow: hidden; line-height: 20px">
                {'I' + pin}<br /><small>{io.getInputName(io.port, pin)}</small>
              </button>
            {/each}
          </div>
        </div>
      </div>
    </div>
  </TaskRunnerModalLayout>
{/if}
