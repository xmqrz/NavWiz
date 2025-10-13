<script>
  import taskTemplates from '$lib/shared/services/config/task-templates';

  export let curMode;

  const activeClass = 'variant-filled-secondary pointer-events-none';
</script>

<div class="variant-filled btn-group mb-5">
  <a
    class={curMode === 'global-param' ? activeClass : ''}
    href={taskTemplates.globalParamUrl()}>
    <span class="badge"><i class="fa-solid fa-book"></i></span>
    Global Parameters
  </a>
  <a class={curMode === 'variable' ? activeClass : ''} href={taskTemplates.variableUrl()}>
    <span class="badge"><i class="fa-solid fa-close"></i></span>
    Variables
  </a>
</div>
