<script>
  import { title } from 'stores/page-title';
  import { monoTimeDisplay } from 'stores/server-clock.js';

  export let data;
  export let baseUrl;
  export let pageTitle;
  export let defaultRawUrl;

  $: subtitle = data.logPath ? `(${data.logPath})` : '';
  title.set(pageTitle);
</script>

<div class="p-5">
  <div class="flex py-5">
    <h1 class="text-3xl tracking-widest">{pageTitle} {subtitle}</h1>
    <a class="btn-icon opacity-50" href={data.raw_url || defaultRawUrl}>
      <i class="fa-solid fa-link"></i>
    </a>
  </div>
  <hr />
  <br />
  <div class="w-full">
    <div class="col-md-6">
      <div class="table-container">
        <table class="table-hover w-full font-mono">
          <thead class="text-left">
            <tr>
              <th>Name</th>
              <th>Modified Time</th>
              <th>File Size</th>
            </tr>
          </thead>
          <tbody>
            {#if data.result && data.result.length > 0}
              {#each data.result as d}
                <tr>
                  <td>
                    {#if d.is_directory}
                      <a class="anchor" href="{baseUrl}/{d.name}">
                        {d.name}
                      </a>
                    {:else}
                      <a class="anchor" href={d.url} download={d.name}>
                        {d.name}
                      </a>
                    {/if}
                  </td>
                  <td>
                    {monoTimeDisplay(d.timestamp)}
                  </td>
                  <td>
                    {d.size}
                  </td>
                </tr>
              {/each}
            {:else}
              <tr><td colspan="8">No log yet.</td></tr>
            {/if}
          </tbody>
        </table>
      </div>
    </div>
  </div>
</div>
