import { error } from '@sveltejs/kit';
import { get } from 'svelte/store';

import { unauthorized, getCSRFToken } from 'stores/auth.js';
import { sockjsConnected } from 'stores/states.js';
import PinElevationModal from '$lib/modal/PinElevationModal.svelte';
import UserPassElevationModal from '$lib/modal/UserPassElevationModal.svelte';

class FetchError extends Error {
  constructor(message, errorCode, response = null) {
    super(message, { cause: response });
    this.errorCode = errorCode;
  }
}

let modalStore;
export function initHttp(ms) {
  modalStore = ms;
}

export async function getAPI(url, fetch = window.fetch, method = 'GET') {
  try {
    const response = await fetch(url.startsWith('http') ? url : API_URL + url, {
      method: method
    });
    if (response.ok) {
      const result = await response.json();
      return result;
    } else if (response.status === 401) {
      // Unauthorized
      unauthorized(window.location.pathname);
    } else {
      throw new FetchError(response.statusText, response.status, response);
    }
  } catch (err) {
    if (err instanceof FetchError) {
      if (err.errorCode >= 500 && err.errorcode < 600 && !get(sockjsConnected)) {
        throw error(err.errorCode, {
          message: 'Backend Offline'
        });
      } else if (err.errorCode === 406) {
        let fms_dashboard = '';
        try {
          const data = await err.cause.json();
          fms_dashboard = data.fms_dashboard;
        } catch (error) {
          console.log(`Fail to parse error feedback ${err.cause}`);
        }
        throw error(err.errorCode, {
          data: fms_dashboard
        });
      } else if (err.errorCode === 403) {
        let detail = '';
        try {
          const data = await err.cause.json();
          detail = data.detail;
        } catch (error) {
          console.log(`Fail to parse error feedback ${err.cause}`);
        }
        const licenseVoid = detail.includes('license');

        // TODO: if permission denied should we refresh auth permission store incse permission changed.?
        throw error(err.errorCode, {
          data: detail,
          licenseVoid,
          title: licenseVoid ? 'License Error' : undefined
        });
      }
    }
    console.error(`Fail to get ${url} data:`, err);
  }
}

export async function getAllAPI(url, cb = undefined, fetch = window.fetch) {
  // break if callback return true
  // callback is called with response, curResults and last flag for each request.
  let results = [];
  while (url) {
    const response = await getAPI(url, fetch);
    url = undefined;
    if (response) {
      url = response.next;
      if (Array.isArray(response.results)) {
        results = results.concat(response.results);
      }
    }
    if (cb && cb(response, results, !url)) {
      break;
    }
  }

  return results;
}

export async function postAPI(url, data, method, fetch, raw) {
  return _postAPI(url, data, method, fetch, raw);
}

export async function elevatedPostAPI(url, data, method, fetch, raw) {
  return _postAPI(url, data, method, fetch, raw, true);
}

// allowElevate may return { cancelled: true } for cancelled or failed elevation.
async function _postAPI(
  url,
  data,
  method = 'POST',
  fetch = window.fetch,
  raw = false,
  allowElevate = false,
  elevated = ''
) {
  try {
    const csrftoken = getCSRFToken();
    let headers = {
      'Content-Type': 'application/json',
      'X-CSRFToken': csrftoken
    };
    if (elevated) {
      headers['Authorization'] = elevated;
    }
    const response = await fetch(API_URL + url, {
      method,
      headers,
      body: JSON.stringify(data)
    });
    if (response.ok) {
      const result = raw ? await response.text() : await response.json();
      return result;
    } else if (response.status === 401 && !allowElevate) {
      // NOTE: (!allowElevate) to prevent redirect for elevated error.
      // Unauthorized
      unauthorized(window.location.pathname);
    } else {
      throw new FetchError(response.statusText, response.status, response);
    }
  } catch (err) {
    console.error(`Fail to post ${url} data:`, err);
    if (err instanceof FetchError) {
      if (err.errorCode === 403 || err.errorCode === 401) {
        // retry using pin
        if (allowElevate && !elevated) {
          return new Promise(function (resolve, reject) {
            // Note: avoid modal flickering since out transition triggered first before this can be added.
            sleep(150).then(() => {
              modalStore.priorityTrigger({
                type: 'component',
                position: 'items-center',
                magic: true,
                component: {
                  ref: window.agvPanelToken ? PinElevationModal : UserPassElevationModal
                },
                response: async (r) => {
                  // Note: avoid modal flickering if modal added after resolve.
                  await sleep(150);
                  if (!r) {
                    if (r !== undefined) {
                      reject(err);
                    } else {
                      resolve({
                        cancelled: true
                      });
                    }
                    return;
                  }
                  resolve(_postAPI(url, data, method, fetch, raw, allowElevate, r));
                }
              });
            });
          });
        } else if (allowElevate && elevated) {
          let msg = err.message;
          try {
            const d = await err.cause.json();
            if (d.detail) {
              msg = d.detail;
            }
          } catch {
            console.log('fail to parse error reason in api elevation');
          }
          modalStore.trigger({
            type: 'alert',
            title: 'Elevation Failed',
            body: msg.replace(/\n/g, '<br/>'),
            buttonTextCancel: 'Ok'
          });
          return { cancelled: true };
        }
      }
    }

    throw err;
    // TODO: if backend fail we abort or we keep retry here. (will display loading always..)
  }
}

async function sleep(ms) {
  await new Promise((resolve) => setTimeout(resolve, ms));
}

export async function formAPI(url, form, method = 'POST', fetch = window.fetch) {
  try {
    const csrftoken = getCSRFToken();
    let form_data = form instanceof FormData ? form : new FormData(form);
    const response = await fetch(API_URL + url, {
      headers: {
        'X-CSRFToken': csrftoken
      },
      method: method,
      body: form_data
    });
    if (response.ok) {
      const result = await response.json();
      return result;
    } else if (response.status === 401) {
      // Unauthorized
      unauthorized(window.location.pathname);
    } else {
      throw new FetchError(response.statusText, response.status, response);
    }
  } catch (err) {
    console.error(`Fail to post ${url} data:`, err);
    throw err;
    // TODO: if backend fail we abort or we keep retry here. (will display loading always..)
  }
}

export async function downloadFileAPI(url, data, method = 'POST') {
  const form = document.createElement('form');
  form.setAttribute('method', method);
  form.setAttribute('action', API_URL + url + '?format=json');
  const csrftoken = getCSRFToken();
  const input = document.createElement('input');
  input.setAttribute('type', 'hidden');
  input.setAttribute('name', 'csrfmiddlewaretoken');
  input.setAttribute('value', csrftoken);
  form.appendChild(input);
  for (let key in data) {
    const input = document.createElement('input');
    input.setAttribute('type', 'hidden');
    input.setAttribute('name', key);
    input.setAttribute('value', data[key]);
    form.appendChild(input);
  }
  let iframe = document.getElementById('iframe-agv05-api');
  if (!iframe) {
    iframe = document.createElement('iframe');
    iframe.id = 'iframe-agv05-api';
    iframe.style.display = 'none';
    document.body.appendChild(iframe);
  }
  iframe.onload = function () {
    // Download success will not trigger this event.
    const iframeDoc = iframe.contentDocument || iframe.contentWindow.document;
    try {
      const raw = iframeDoc.body.querySelector(':first-child').innerText;
      const resp = JSON.parse(raw);
      alert(
        `Fail to download: ${resp.detail || resp.error || resp.status || 'server error.'}`
      );
    } catch (error) {
      console.log('fail to parse download error response in downloadFileAPI.');
      console.error('Error:', error);
      alert('Fail to download: server error.');
    }
    iframeDoc.body.innerHTML = '';
  };
  var iframeDoc = iframe.contentDocument || iframe.contentWindow.document;
  iframeDoc.body.innerHTML = '';
  iframeDoc.body.appendChild(form);
  form.submit();
  iframeDoc.body.removeChild(form);
}
