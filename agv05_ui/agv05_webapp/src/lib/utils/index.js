import * as _ from 'lodash-es';

export {
  initHttp,
  getAPI,
  getAllAPI,
  postAPI,
  elevatedPostAPI,
  formAPI,
  downloadFileAPI
} from './http.js';
export { initTooltip } from './tooltip.js';
export { floatingType, floating } from './floating.js';
export { extendModalStore } from './modal-store.js';

export function allBools(params) {
  return params.length >= 3 && _.every(params, (p) => typeof p.value === 'boolean');
}

export function formatMinMax(param) {
  var display;
  if (param.type === 'int') {
    display = 'Min: ';
    display += _.isUndefined(param.min) ? '-' : param.min;
    display += ' ; Max: ';
    display += _.isUndefined(param.max) ? '-' : param.max;
    return display;
  } else if (param.type === 'double') {
    display = 'Min: ';
    display += _.isUndefined(param.min) ? '-' : param.min;
    display +=
      !_.isUndefined(param.min) && param.min.toString().indexOf('.') === -1 ? '.0' : '';
    display += ' ; Max: ';
    display += _.isUndefined(param.max) ? '-' : param.max;
    display +=
      !_.isUndefined(param.max) && param.max.toString().indexOf('.') === -1 ? '.0' : '';
    return display;
  }
}

const urlMap = {
  agv: '/config/agv',
  'task-template-global-param': '/config/task-templates/global-param',
  variable: '/config/task-templates/variable',
  'map-list': '/config/maps',
  'map-active': '/config/maps/active',
  'map-param': '/config/maps/param',
  'map-teleport': '/config/maps/teleport',
  'map-transition-trigger': '/config/maps/transition-trigger'
};

export function resourceIdToUrl(resourceId, item = undefined) {
  // TODO: standardize url query for search and validation url.
  if (resourceId in urlMap) {
    if (item) {
      return `${urlMap[resourceId]}?filter=${item}`;
    }
    return urlMap[resourceId];
  } else if (resourceId.startsWith('_map_')) {
    let mapID = resourceId.substring(5);
    if (item) {
      return `/config/maps/${mapID}/layout?search=${item}`;
    }
    return `/config/maps/${mapID}/edit`;
  } else if (resourceId.startsWith('_ttpk_')) {
    let ttpk = resourceId.substring(6);
    if (item) {
      return `/config/task-templates/${ttpk}/edit?search=${item}`;
    }
    return `/config/task-templates/${ttpk}/edit`;
  }
}

export function scrollTop() {
  const elemPage = document.querySelector('#page');
  if (elemPage !== null) {
    elemPage.scrollTop = 0;
  }
}

export function defaultdict(initial, defaultValue) {
  return new Proxy(initial, {
    get: function (target, prop) {
      if (!(prop in target)) {
        return typeof defaultValue === 'function' ? defaultValue() : defaultValue;
      }
      return target[prop];
    }
  });
}
