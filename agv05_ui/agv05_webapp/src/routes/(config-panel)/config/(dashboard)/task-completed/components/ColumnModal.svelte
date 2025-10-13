<script>
  import { getModalStore } from '@skeletonlabs/skeleton';

  // Props
  // svelte-ignore unused-export-let
  export let parent;

  const modalStore = getModalStore();

  let selection = [
    'name',
    'template',
    'params',
    'status',
    'creationTime',
    'startTime',
    'scheduledStartTime',
    'endTime',
    'duration',
    'owner',
    'diagnostics'
  ];

  if (Array.isArray($modalStore[0].init)) {
    selection = $modalStore[0].init;
  }

  const childRelation = {
    template: ['params'],
    startTime: ['scheduledStartTime']
  };

  function updateSelection() {
    // Avoid assignment due to recursive reactive call.
    for (const [k, v] of Object.entries(childRelation)) {
      if (!selection.includes(k)) {
        for (const val of v) {
          const index = selection.indexOf(val);
          if (index >= 0) {
            selection.splice(index, 1);
          }
        }
      }
    }
  }

  $: updateSelection(selection);

  function onApply() {
    if ($modalStore[0].response) {
      $modalStore[0].response({ selection });
    }
    modalStore.close();
  }
</script>

{#if $modalStore[0]}
  <div class="card w-full max-w-[500px] space-y-2 p-6 pl-10">
    <label class="flex items-center space-x-2">
      <input type="checkbox" class="checkbox" bind:group={selection} value="name" />
      <p>Name</p>
    </label>
    <label class="flex items-center space-x-2">
      <input type="checkbox" class="checkbox" bind:group={selection} value="template" />
      <p>Template</p>
    </label>
    <label class="ml-7 flex items-center space-x-2">
      <input
        type="checkbox"
        class="checkbox"
        bind:group={selection}
        value="params"
        disabled={!selection.includes('template')} />
      <p>Params</p>
    </label>
    <label class="flex items-center space-x-2">
      <input type="checkbox" class="checkbox" bind:group={selection} value="status" />
      <p>Status</p>
    </label>
    <label class="flex items-center space-x-2">
      <input type="checkbox" class="checkbox" bind:group={selection} value="creationTime" />
      <p>Creation Time</p>
    </label>
    <label class="flex items-center space-x-2">
      <input type="checkbox" class="checkbox" bind:group={selection} value="startTime" />
      <p>Start Time</p>
    </label>
    <label class="ml-7 flex items-center space-x-2">
      <input
        type="checkbox"
        class="checkbox"
        bind:group={selection}
        value="scheduledStartTime"
        disabled={!selection.includes('startTime')} />
      <p>Scheduled Start Time</p>
    </label>
    <label class="flex items-center space-x-2">
      <input type="checkbox" class="checkbox" bind:group={selection} value="endTime" />
      <p>End Time</p>
    </label>
    <label class="flex items-center space-x-2">
      <input type="checkbox" class="checkbox" bind:group={selection} value="duration" />
      <p>Duration</p>
    </label>
    <label class="flex items-center space-x-2">
      <input type="checkbox" class="checkbox" bind:group={selection} value="owner" />
      <p>Owner</p>
    </label>
    <label class="flex items-center space-x-2">
      <input type="checkbox" class="checkbox" bind:group={selection} value="diagnostics" />
      <p>Diagnostics</p>
    </label>

    <footer class="card-footer flex justify-end">
      <button type="button" class="variant-filled-surface btn" on:click={onApply}>
        Apply
      </button>
    </footer>
  </div>
{/if}
