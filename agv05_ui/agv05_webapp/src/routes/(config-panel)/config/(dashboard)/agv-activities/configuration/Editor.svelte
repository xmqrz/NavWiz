<script>
  export let activities;
  export let activityLabels;
  export let updateActivityLabel;
</script>

<div class="table-container">
  <table class="table">
    <thead>
      <tr class="bg-secondary-400">
        <th colspan="3">Activity Labels</th>
      </tr>
      <tr>
        <th>Activity</th>
        <th>Activity Code</th>
        <th>Label (for reporting)</th>
      </tr>
    </thead>
    <tbody>
      {#each activities as ac}
        <tr>
          <td>{ac[1]}</td>
          <td>{ac[0]}</td>
          <td>
            <input
              class="input"
              type="text"
              placeholder={ac[1]}
              value={activityLabels[ac[0]] || ''}
              on:change={(evt) => updateActivityLabel(ac[0], evt.target.value)} />
          </td>
        </tr>
      {/each}
    </tbody>
  </table>
</div>
