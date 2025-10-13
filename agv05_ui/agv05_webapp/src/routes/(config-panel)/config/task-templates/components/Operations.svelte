<script>
  import taskTemplates from '$lib/shared/services/config/task-templates';

  export let selected = [];
</script>

{#if selected.length > 1}
  <div class="inline-block pl-5">
    <span class="variant-filled chip h-7">{selected.length} items selected</span>
    <a
      href={taskTemplates.massUpdateUrl(selected)}
      class="variant-filled btn mb-1 h-7 w-7 rounded p-1"
      tip-title="Edit">
      <i class="fa-solid fa-pencil"></i>
    </a>
    <a
      href={taskTemplates.massCategoryUpdateUrl(selected)}
      class="variant-filled btn mb-1 h-7 w-7 rounded p-1"
      tip-title="Edit Category">
      <i class="fa-solid fa-tags"></i>
    </a>
    <a
      href={taskTemplates.massUsersUpdateUrl(selected)}
      class="variant-filled btn mb-1 h-7 w-7 rounded p-1"
      tip-title="Users">
      <i class="fa-solid fa-users"></i>
    </a>
    <a
      href={taskTemplates.massDeleteUrl(selected)}
      class="variant-filled btn mb-1 h-7 w-7 rounded p-1"
      tip-title="Delete">
      <i class="fa-solid fa-times"></i>
    </a>
  </div>
{/if}
