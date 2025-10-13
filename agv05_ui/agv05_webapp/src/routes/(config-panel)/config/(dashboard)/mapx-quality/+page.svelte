<script>
  import ConfigLayout from 'components/ConfigLayout.svelte';
  import editorF from './mapx-quality/editor';

  export let data;

  let qmaps = data.mqs.qmaps;
  let selectedFileName;
  let media_url = data.mqs.media_url;

  function formatDateTime(dateTimeString) {
    // Parse the date string
    let date = new Date(dateTimeString);

    // Extract the date components
    let year = date.getFullYear();
    let month = String(date.getMonth() + 1).padStart(2, '0');
    let day = String(date.getDate()).padStart(2, '0');

    // Extract the time components
    let hours = String(date.getHours()).padStart(2, '0');
    let minutes = String(date.getMinutes()).padStart(2, '0');
    let seconds = String(date.getSeconds()).padStart(2, '0');

    // Construct the formatted date-time string
    let formattedDateTime = `${year}-${month}-${day}  ${hours}:${minutes}:${seconds}`;

    return formattedDateTime;
  }
</script>

<ConfigLayout title="Map Quality Score Page" validation={false}>
  <div class="row">
    <div class="col-md-12">
      <section id="form" class="agv05x-map agv05x-map-quality" resizable="true" grid="true">
        <form class="space-y-2">
          <div class="grid w-full grid-cols-7 gap-4">
            <div class="col-span-1 text-right">
              <label for="id_qmap">Session</label>
            </div>
            <div class="col-span-6 space-y-2">
              <select id="id_qmap" name="qmap" class="select" bind:value={selectedFileName}>
                {#if qmaps}
                  {#each qmaps as [filename, start, end]}
                    <option value={filename}>
                      {formatDateTime(start)} &mdash; {formatDateTime(end)}
                    </option>
                  {/each}
                {:else}
                  <option hidden value="">-- No sessions yet --</option>
                {/if}
              </select>
            </div>
          </div>
          {#if qmaps}
            <div class="grid w-full grid-cols-7 gap-4">
              <div class="col-span-1 text-right">
                <label for="id_statistic">Statistic</label>
              </div>
              <div class="col-span-6 space-y-2">
                <select id="id_statistic" name="statistic" class="select">
                  <option value="avg">Average</option>
                  <option value="max">Maximum</option>
                  <option value="min">Minimum</option>
                </select>
              </div>
            </div>
          {/if}
          <input id="id_media_url" name="media_url" value={media_url} type="hidden" />
          <div class="editor" use:editorF></div>
        </form>
      </section>
    </div>
  </div>
</ConfigLayout>

<style>
  @import '$lib/styles/viz.css';
  @import '$lib/styles/map-viz.css';
</style>
