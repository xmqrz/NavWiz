import { error, redirect } from '@sveltejs/kit';
import { writable, readonly, get } from 'svelte/store';
import { goto } from '$app/navigation';
import { t } from '$lib/translations';

// TODO: listen to localstorage and makesure all instance logout when user logout...

function has_perms(perms) {
  const $user = this;

  if ($user.is_superuser) {
    return true;
  }

  if (!Array.isArray(perms)) {
    perms = [perms];
  }

  if ($user.permissions && perms.every((p) => $user.permissions.includes(p))) {
    return true;
  }

  return false;
}

function has_perms_with_pin(perms) {
  const $user = this;

  if (!$user.permissions_with_pin) {
    return false;
  }

  if ($user.is_superuser) {
    return true;
  }

  if (!Array.isArray(perms)) {
    perms = [perms];
  }

  if (perms.every((p) => $user.permissions_with_pin.includes(p))) {
    return true;
  }

  return false;
}

function is_agv_panel() {
  const $user = this;
  return ['agv_panel', 'agv_panel_pin_protected'].indexOf($user.username) >= 0;
}

const defaultUser = {
  has_perms,
  has_perms_with_pin,
  is_agv_panel,
  is_superuser: false,
  username: 'Guest',
  group: 'Guest',
  login: false,
  permissions: [],
  username_with_pin: undefined,
  permissions_with_pin: [],
  env: {}
};
const userInfo = writable(Object.assign({}, defaultUser));
export const user = readonly(userInfo);

let authInitResolve;
let authInitReject;
let authInit = new Promise((resolve, reject) => {
  authInitResolve = resolve;
  authInitReject = reject;
});

export function updateUserInfo(info) {
  userInfo.update((value) => {
    let v = Object.assign(value, info);

    // Stub to simulate retrieving DFleet permissions
    v.permissions = [
      ...v.permissions,
      'agvccs.add_task',
      'agvccs.add_multiple_tasks',
      'agvccs.abort_own_task',
      'agvccs.cancel_own_task',
      'agvccs.prioritize_own_task',
      'agvccs.suspend_own_task',
      'agvccs.resume_own_task'
    ];
    return v;
  });
}

export const initAuth = async (fetch) => {
  try {
    let response;
    if (window.agvPanelToken) {
      response = await fetch(SYSTEM_URL + '/agv-panel-login', {
        method: 'POST',
        headers: {
          Authorization: `Token ${window.TRACKLESS ? window.agvPanelTokenX : window.agvPanelToken}`,
          'Content-Type': 'application/json'
        }
      });
    } else {
      response = await fetch(SYSTEM_URL + '/auth');
    }
    if (response.ok) {
      const result = await response.json();
      updateUserInfo(result);
      if (authInitResolve) {
        authInitResolve();
        authInitResolve = undefined;
        authInitReject = undefined;
      }
    } else {
      throw new Error(response.statusText);
    }
  } catch (error) {
    console.error('Fail to get backend auth info:', error);
    if (authInitReject) {
      authInitReject();
      authInitResolve = undefined;
      authInitReject = undefined;
    }
  }
};

export const noPermissionError = () => {
  return error(403, {
    data: 'You do not have permission to perform this action.'
  });
};

export const pagePermissions = async (perms) => {
  await authInit;

  let $user = get(user);

  if ($user.is_superuser) {
    return;
  }

  if (!$user.permissions || !perms.every((p) => $user.permissions.includes(p))) {
    if (!$user.login) {
      throw redirect(302, `/login?next=${encodeURIComponent(window.location.pathname)}`);
    }

    throw noPermissionError();
  }
};

export const login = async (username, password) => {
  try {
    const csrftoken = getCSRFToken();
    const options = {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'X-CSRFToken': csrftoken
      },
      body: JSON.stringify({ username, password })
    };
    const response = await fetch(SYSTEM_URL + '/login', options);
    if (!response.ok) {
      const data = await response.json();
      return data.error || t.get('common.login_fail');
    }
    const data = await response.json();
    updateUserInfo(data);
    // TODO: call invalidate
  } catch (error) {
    console.log(`Fail to login: ${error}`);
    return t.get('common.login_fail');
  }
};

export const logout = async () => {
  try {
    const response = await fetch(SYSTEM_URL + '/logout');
    if (response.ok) {
      const result = await response.json();
      updateUserInfo(result);
      goto('/');
    }
  } catch (error) {
    console.log(`Fail to logout: ${error}`);
  }
};

export const unauthorized = (nextUrl) => {
  updateUserInfo(defaultUser);
  goto(`/login?next=${encodeURIComponent(nextUrl)}`, { invalidateAll: true });
};

export function getCSRFToken() {
  return getCookie(getEnv('CSRFNAME'));
}

export function getEnv(key) {
  let $user = get(user);
  return $user.env[key];
}

function getCookie(name) {
  const cookieValue = document.cookie.match('(^|;) ?' + name + '=([^;]*)(;|$)');
  return cookieValue ? decodeURIComponent(cookieValue[2]) : null;
}
