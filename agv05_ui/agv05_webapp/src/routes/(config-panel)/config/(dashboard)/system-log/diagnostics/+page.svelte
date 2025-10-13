<script>
  import { title } from 'stores/page-title';
  import { timeDisplay } from 'stores/server-clock.js';

  import editorF from './diagnostic-view';

  export let data;
  title.set('Diagnostics');
</script>

<div class="p-5">
  <div class="flex py-5">
    <h1 class="text-3xl tracking-widest">Diagnostics</h1>
    <a class="btn-icon opacity-50" href="/logs/diagnostics/">
      <i class="fa-solid fa-link"></i>
    </a>
  </div>
  <hr />
  <br />
  <div class="w-full">
    <div class="editor agv05-diagnostic-view col-md-6" use:editorF>
      <div class="table-container">
        <table class="table table-hover">
          <thead>
            <tr>
              <th width="30px"> # </th>
              <th>Name</th>
              <th>Modified Time</th>
              <th>File Size</th>
              <th>Operation</th>
            </tr>
          </thead>
          <tbody>
            {#if data.diagnosticsList && data.diagnosticsList.length > 0}
              {#each data.diagnosticsList as d, i}
                <tr>
                  <td>
                    {i + 1}
                  </td>
                  <td>
                    {d.filename}
                  </td>
                  <td>
                    {timeDisplay(d.timestamp)}
                  </td>
                  <td>
                    {d.size}
                  </td>
                  <td>
                    <button
                      type="button"
                      data-url={d.url}
                      class="btn-xs btn-default diagnostic-start-view btn"
                      tip-title="View Diagnostics">
                      <i class="fa fa-table"></i>
                    </button>
                    <a
                      href={d.url}
                      class="btn-xs btn-default btn"
                      tip-title="Download Diagnostics"><i class="fa fa-download"></i></a>
                  </td>
                </tr>
              {/each}
            {:else}
              <tr><td colspan="8">No diagnostic yet.</td></tr>
            {/if}
          </tbody>
        </table>
      </div>
    </div>
  </div>
</div>
