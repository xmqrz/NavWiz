<script>
  import { page } from '$app/stores';
  import ConfigLayout from 'components/ConfigLayout.svelte';
  import maint from '$lib/shared/services/config/maintenance';

  const activeClass = 'variant-filled-secondary pointer-events-none';
  const tabs = [
    {
      label: 'AMR & Parts',
      href: maint.url()
    },
    {
      label: 'Preventive Maintenance',
      href: maint.pmUrl()
    },
    {
      label: 'Service Logs',
      href: maint.serviceLogsUrl()
    }
  ];
</script>

<ConfigLayout title="Maintenance Info" validation={false}>
  <div class="variant-filled btn-group mb-5">
    {#each tabs as { label, href }}
      <a class={$page.url.pathname === href && activeClass} {href}>{label}</a>
    {/each}
    <a href={maint.linkUrl()} target="_blank">
      View in DF Hub
      <i class="fa-solid fa-up-right-from-square ml-2"></i>
    </a>
  </div>
  <slot />
</ConfigLayout>
