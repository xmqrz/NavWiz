<script>
  import { getToastStore } from '@skeletonlabs/skeleton';

  import ConfigLayout from 'components/ConfigLayout.svelte';
  import users from '$lib/shared/services/config/users';
  import UserTokenEditor from './UserTokenEditor.svelte';

  export let data;

  const toastStore = getToastStore();

  function onRegen() {
    users
      .regenerateAuthToken(data.user.id)
      .then((d) => {
        toastStore.trigger({
          message: 'Auth token regenerated.',
          timeout: 3000,
          hoverable: true
        });
        data.authToken = d;
      })
      .catch((e) => {
        console.log(e);
        // TODO: handle provide error message.
      });
  }
</script>

<ConfigLayout
  title={`Auth token of user "${data.user.username}"`}
  back={['Users', users.listUrl()]}
  validation={false}>
  <UserTokenEditor bind:data on:regen={onRegen} />
</ConfigLayout>
