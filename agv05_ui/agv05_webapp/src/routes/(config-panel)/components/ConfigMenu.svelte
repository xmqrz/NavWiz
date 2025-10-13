<script>
  import { popup } from '@skeletonlabs/skeleton';

  import Perm from 'components/Perm.svelte';

  const configPopup = {
    event: 'click',
    target: 'configPopup',
    placement: 'bottom-end'
  };
</script>

<Perm perms="system.view_system_panel">
  <button
    type="button"
    class="variant-filled-secondary bg-secondary-400-500-token btn hidden h-[42px] text-black md:flex"
    use:popup={configPopup}>
    <i class="fa-solid fa-gear"></i>
    <span class="hidden lg:inline-block">Configs</span>
    <i class="fa-solid fa-caret-down ml-2"></i>
  </button>
  <div class="card z-50 m-5 mt-1 shadow-xl" data-popup="configPopup">
    <div class="flex">
      <nav class="list-nav">
        <ul class="w-56 space-y-1 p-3">
          <li><a href="/config/maintenance" class="btn w-full">Maintenance Info</a></li>
          <Perm perms="system.change_robot">
            <li><a href="/config/robot-config" class="btn w-full">Robot Configuration</a></li>
          </Perm>
          <Perm perms="system.change_white_label">
            <li><a href="/config/white-label" class="btn w-full">White Label</a></li>
          </Perm>
          <li><hr /></li>
          <li><a href="/config/webhooks" class="btn w-full">Webhooks</a></li>
          <li><a href="/api/v3" class="btn w-full">Rest API</a></li>
        </ul>
      </nav>
      <span class="divider-vertical my-3 flex-grow" />
      <nav class="list-nav">
        <ul class="w-56 space-y-1 p-3">
          <li><a href="/config/backup-restore" class="btn w-full">Backup Restore</a></li>
          <li><a href="/config/network" class="btn w-full">Network Connection</a></li>
          <li><a href="/config/date-time" class="btn w-full">Date Time</a></li>
          <Perm perms="system.view_users">
            <li><hr /></li>
            <li><a href="/config/users" class="btn w-full">User</a></li>
            <li><a href="/config/groups" class="btn w-full">Group</a></li>
            <Perm perms="auth.change_permission">
              <li><a href="/config/permissions" class="btn w-full">Permissions</a></li>
            </Perm>
          </Perm>
        </ul>
      </nav>
      <span class="divider-vertical my-3 flex-grow" />
      <nav class="list-nav">
        <ul class="w-56 space-y-1 p-3">
          <li><a href="/config/agv" class="btn w-full">Agv</a></li>
          <li><a href="/config/audio" class="btn w-full">Audio</a></li>
          <li><a href="/config/io" class="btn w-full">I/O</a></li>
          <li><a href="/config/laser-sensors" class="btn w-full">Laser Sensor</a></li>
          <li><a href="/config/maps" class="btn w-full">Maps</a></li>
          <li><a href="/config/parameters" class="btn w-full">Parameters</a></li>
          <li><a href="/config/task-templates" class="btn w-full">Task Template</a></li>
        </ul>
      </nav>
    </div>
  </div>
</Perm>
