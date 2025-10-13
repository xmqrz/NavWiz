<script>
  import { serverClockFormat } from 'stores/server-clock.js';
  import Perm from 'components/Perm.svelte';

  const time = serverClockFormat('h:mm:ss a');
  const date = serverClockFormat('dd/MM/yyyy');
</script>

<Perm perms="app.show_system_clock">
  <div class="flex min-w-[70px] flex-col text-white md:min-w-[90px]">
    <span class="self-end text-xs md:text-sm">
      {$time}
    </span>
    <span class="self-end text-xs md:text-sm">
      {$date}
    </span>
  </div>
</Perm>
