<script>
  import { remoteIoChannel } from 'stores/sock';
  import { onMount } from 'svelte';
  import { getModalStore } from '@skeletonlabs/skeleton';

  import TaskRunnerModalLayout from './TaskRunnerModalLayout.svelte';

  // Props
  export let parent;
  export let activeModule;

  let remoteIo = new RemoteIo();

  const modalStore = getModalStore();

  function RemoteIo() {
    this._inputs = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    this._outputs = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

    this.getInput = (rio, channel) => this._inputs[rio - 1] & (1 << channel);
    this.getOutput = (rio, channel) => this._outputs[rio - 1] & (1 << channel);
  }

  function updateRemoteIo(data) {
    if (!data) {
      return;
    }
    if (Array.isArray(data.inputs) && data.inputs.length === 10) {
      remoteIo._inputs = data.inputs;
    }
    if (Array.isArray(data.outputs) && data.outputs.length === 10) {
      remoteIo._outputs = data.outputs;
    }
  }

  // callback
  function remoteIoCallback(data) {
    updateRemoteIo(data);
  }

  onMount(() => {
    remoteIo.rio = 1;
    remoteIoChannel.subscribe(remoteIoCallback);

    return () => {
      remoteIoChannel.unsubscribe(remoteIoCallback);
    };
  });

  const gridStretch = 'grid grid-cols-8 place-content-stretch';
</script>

{#if $modalStore[0]}
  <TaskRunnerModalLayout {activeModule}>
    <div class="panel-modal card flex flex-col rounded-3xl">
      <div class="variant-filled-secondary rounded-t-2xl p-4" style="position: relative;">
        <span class="text-center font-bold">Remote I/O (View Only)</span>
        <div class="float-right">
          <button type="button" class="btn btn-sm" on:click={parent.onClose}>
            <i class="fa-solid fa-xmark"></i>
          </button>
        </div>
      </div>
      <div class="grow space-y-2 overflow-auto p-4">
        <div class="grid grid-cols-10 place-content-stretch" style="padding:0">
          {#each [1, 2, 3, 4, 5, 6, 7, 8, 9, 10] as i}
            <button
              type="button"
              class="divide-x divide-solid divide-black p-4 hover:bg-slate-300 dark:text-slate-500 {remoteIo.rio ===
              i
                ? 'bg-slate-300'
                : 'bg-white'}"
              on:click={() => (remoteIo.rio = i)}>#{i}</button>
          {/each}
        </div>
        <div>
          <div class={gridStretch}>
            {#each [15, 14, 13, 12, 11, 10, 9, 8] as channel}
              <button
                type="button"
                class="p-2 {remoteIo.getOutput(remoteIo.rio, channel)
                  ? 'variant-filled-success'
                  : 'variant-filled-error'}"
                style="vertical-align: top; white-space: normal; overflow: hidden; line-height: 20px">
                {'O' + channel}
              </button>
            {/each}
          </div>
          <div class={gridStretch}>
            {#each [7, 6, 5, 4, 3, 2, 1, 0] as channel}
              <button
                type="button"
                class="p-2 {remoteIo.getOutput(remoteIo.rio, channel)
                  ? 'variant-filled-success'
                  : 'variant-filled-error'}"
                style="vertical-align: top; white-space: normal; overflow: hidden; line-height: 20px">
                {'O' + channel}
              </button>
            {/each}
          </div>
        </div>
        <div>
          <div class={gridStretch}>
            {#each [15, 14, 13, 12, 11, 10, 9, 8] as channel}
              <button
                type="button"
                class="p-2 {remoteIo.getInput(remoteIo.rio, channel)
                  ? 'variant-filled-success'
                  : 'variant-filled-error'}"
                style="vertical-align: top; white-space: normal; overflow: hidden; line-height: 20px">
                {'I' + channel}
              </button>
            {/each}
          </div>
          <div class={gridStretch}>
            {#each [7, 6, 5, 4, 3, 2, 1, 0] as channel}
              <button
                type="button"
                class="p-2 {remoteIo.getInput(remoteIo.rio, channel)
                  ? 'variant-filled-success'
                  : 'variant-filled-error'}"
                style="vertical-align: top; white-space: normal; overflow: hidden; line-height: 20px">
                {'I' + channel}
              </button>
            {/each}
          </div>
        </div>
      </div>
    </div>
  </TaskRunnerModalLayout>
{/if}
