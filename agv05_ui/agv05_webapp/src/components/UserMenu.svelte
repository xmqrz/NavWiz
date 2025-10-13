<script>
  import { getDrawerStore, getModalStore, popup } from '@skeletonlabs/skeleton';
  import { getContext } from 'svelte';
  import { page } from '$app/stores';

  import { logout } from 'stores/auth.js';
  import { t } from '$lib/translations';
  import Perm from 'components/Perm.svelte';
  import users from '$lib/shared/services/config/users';

  let className = '';
  export { className as class };

  const drawerStore = getDrawerStore();
  const modalStore = getModalStore();

  const userPopup = {
    event: 'click',
    target: 'userPopup',
    placement: 'bottom'
  };

  const user = getContext('user');

  function handleLogin() {
    drawerStore.open({
      id: 'login'
    });
  }

  function handleLogout() {
    const modal = {
      type: 'confirm',
      // Data
      title: 'Logout NavWiz',
      body: 'Are you sure you wish to logout?',
      response: (r) => {
        if (r) {
          logout();
        }
      }
    };
    modalStore.trigger(modal);
  }
</script>

<Perm perms="app.show_system_icon">
  <button
    type="button"
    class="variant-filled-surface btn h-[42px] {className}"
    use:popup={userPopup}>
    <i class="fa-solid fa-user md:mr-2"></i>
    <span class="hidden md:inline">{$user.username}</span>
    <i class="fa-solid fa-caret-down ml-2 hidden md:inline"></i>
  </button>
  <div class="z-50" data-popup="userPopup">
    <div class="card mr-6 mt-1 p-4 shadow-xl">
      <nav class="list-nav">
        <ul class="space-y-1">
          <slot />
          {#if $user.login}
            <Perm perms="system.change_own_password">
              <li>
                <a href={users.changeOwnPasswordUrl($page.url.pathname)} class="btn w-full">
                  {$t('common.change_password')}
                </a>
              </li>
              <li>
                <a href={users.myAuthTokenUrl($page.url.pathname)} class="btn w-full">
                  My Auth Token
                </a>
              </li>
            </Perm>
            <hr />
          {/if}
          <li>
            {#if $user.login}
              <button type="button" class="btn w-full" on:click={handleLogout}>
                {$t('common.logout')}
              </button>
            {:else}
              <button type="button" class="btn w-full" on:click={handleLogin}>
                {$t('common.login')}
              </button>
            {/if}
          </li>
        </ul>
      </nav>
    </div>
  </div>

  <svelte:fragment slot="no-perms">
    {#if $user.username !== 'agv_panel'}
      <button type="button" class="variant-filled-surface btn">
        <i class="fa-solid fa-user mr-2"></i>
        {$user.username}
      </button>
    {/if}
  </svelte:fragment>
</Perm>
