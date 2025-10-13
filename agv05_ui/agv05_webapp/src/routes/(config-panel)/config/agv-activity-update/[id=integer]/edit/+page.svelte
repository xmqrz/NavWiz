<script>
  import { getToastStore } from '@skeletonlabs/skeleton';
  import { goto, invalidateAll, beforeNavigate } from '$app/navigation';

  import ConfigLayout from 'components/ConfigLayout.svelte';
  import { timeDisplay } from 'stores/server-clock.js';
  import agvActivities from '$lib/shared/services/config/agv-activities';

  export let data;

  let dirty = false;

  const toastStore = getToastStore();

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

  function onSubmit() {
    agvActivities
      .update(data.activity.id, {
        activity: data.activity.activity
      })
      .then((_d) => {
        toastStore.trigger({
          message: agvActivities.successUpdateActivityMsg(),
          timeout: 3000,
          hoverable: true
        });
        dirty = false;
        goto(data.next ? data.next : agvActivities.listUrl()).then(() => invalidateAll());
      })
      .catch((e) => {
        console.log(e);
        // TODO: handle provide error message.
      });
  }
</script>

<ConfigLayout
  title="Edit agv activity"
  back={['AGV Activities', data.next ? data.next : '/config/agv-activities']}>
  <form action="">
    <div class="grid">
      <div class="grid grid-cols-4 p-3">
        <span class="font-medium">Start time</span>
        <span>{timeDisplay(data.activity.start)}</span>
      </div>
      <div class="grid grid-cols-4 p-3">
        <span class="font-medium">End time</span>
        <span>{timeDisplay(data.activity.end)}</span>
      </div>
      <div class="grid grid-cols-4 p-3">
        <span class="font-medium">Duration</span>
        <span>{data.activity.activity_duration}</span>
      </div>
      <div class="grid grid-cols-4 p-3">
        <span class="font-medium">Activity</span>
        <select
          required
          class="select"
          bind:value={data.activity.activity}
          on:change={triggerDirty}>
          {#each data.activityChoices as choice}
            <option value={choice[0]}>{choice[1]}</option>
          {/each}
        </select>
      </div>
      <div class="p-3">
        <button
          type="submit"
          class="variant-filled-primary btn"
          on:click|preventDefault={onSubmit}
          disabled={!dirty}>
          Submit
        </button>
      </div>
    </div>
  </form>
</ConfigLayout>
