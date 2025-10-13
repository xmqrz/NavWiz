/*
 * Copyright (c) 2024, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar, Tan Kui An Andrew
 */

import $ from 'cash-dom';
import { popup } from '@skeletonlabs/skeleton';
import Select from 'components/Select.svelte';

var toolbarHtml = `
<div class="diagnostic-table-view-toolbar p-3">
  <div class="flex items-center space-x-2">
    <div class="text-xs w-min toolbar-filename"></div>
    <div class="dropdown">
      <button type="button" class="btn btn-xs btn-default p-1 highlight-menu">
        <i class="fa fa-list"></i>
      </button>
      <div class="z-50 bg-slate-100 rounded-md" data-popup="highlight-menu-popup">
        <ul class="bg-surface-900-50-token rounded list-nav variant-filled">
          <li><button type="button" class="highlight-reset w-full">Clear</button></li>
          <li><button type="button" class="highlight-nav w-full">Navigation</button></li>
        </ul>
      </div>
    </div>
    <button type="button" class="btn btn-default btn-xs toolbar-download p-1" title="Download Diagnostic">
      <i class="fa fa-download"></i>
    </button>
    <div class="flex flex-row grow gap-2">
      <div class="grow"></div>
      <div class="column-navigator flex justify-end">
        <span class="text-xs place-content-center">Column Navigator: </span>
      </div>
      <div class="column-selector flex justify-end">
        <span class="text-xs place-content-center">Column Selector: </span>
      </div>
      <button type="button" class="btn btn-default btn-xs toolbar-close-panel p-1" title="Close Panel">
        <i class="fa fa-close"></i>
      </button>
    </div>
  </div>
`;

export default function (viz) {
  var toolbar = $(toolbarHtml);
  viz.append(toolbar);
  var optionList = [];
  var allList = [];

  toolbar.find('.toolbar-close-panel').on('click', function () {
    viz.close();
  });

  toolbar.find('.highlight-reset').on('click', function () {
    viz.table.setNavHint(false);
  });

  toolbar.find('.highlight-nav').on('click', function () {
    viz.table.setNavHint(true);
  });

  toolbar.find('.toolbar-download').on('click', function () {
    let filename = viz.models.rawFilename();
    filename =
      filename.indexOf('csv') >= 0
        ? filename.substring(0, filename.indexOf('csv') + 3)
        : filename + '.csv';
    let data = viz.models.getData([]);
    let csvContent = data.exportCsv();
    let blob = new Blob([csvContent], {
      type: 'text/csv'
    });
    let url = window.URL.createObjectURL(blob);
    let link = $('<a>');
    link.attr('href', url);
    link.attr('download', filename);
    link.hide().appendTo(toolbar);
    link[0].click();
    setTimeout(() => {
      window.URL.revokeObjectURL(url);
      link.remove();
    }, 150);
  });

  let columnSelectorCurrentValue = [];

  function loaded() {
    toolbar.find('.toolbar-filename').text(viz.models.rawFilename());

    let headers = viz.models.headers();
    for (let idx in headers) {
      let header = headers[idx];
      optionList.push([idx, header]);
      allList.push(idx);
    }

    columnSelectorCurrentValue = allList;

    var columnSelectorTarget = toolbar.find('.column-selector');
    var columnNavigatorTarget = toolbar.find('.column-navigator');

    var columnNavigator = new Select({
      target: columnNavigatorTarget[0],
      props: {
        class: 'text-xs w-[400px]',
        buttonClass: 'rounded py-1 text-sm variant-form h-7',
        buttonTitle: 'Column Navigator',
        enableFilter: true,
        multiple: false,
        value: '',
        options: optionList
      }
    });

    columnNavigator.$on('change', function () {
      let column = columnNavigator.value;
      let columns = columnSelectorCurrentValue;
      if (!Array.isArray(columns)) {
        columns = [columns];
      }
      if (columns.indexOf(column) < 0) {
        columns.push(column);
        columnSelector.value = columns;
      }
      viz.table.scrollToColumn(column);
      viz.table.setColumns(columns);
    });

    var columnSelector = new Select({
      target: columnSelectorTarget[0],
      props: {
        class: 'text-xs w-[400px]',
        buttonClass: 'rounded py-1 text-sm variant-form h-7',
        buttonTitle: 'Column Selector',
        enableFilter: true,
        multiple: true,
        value: columnSelectorCurrentValue,
        options: optionList
      }
    });

    columnSelector.$on('change', function () {
      let columns = columnSelector.value;
      if (!Array.isArray(columns)) {
        columns = [columns];
      }
      viz.table.setColumns(columns);
      columnSelectorCurrentValue = columns;
    });

    var highlightMenu = toolbar.find('.highlight-menu');
    popup(highlightMenu[0], {
      event: 'click',
      target: 'highlight-menu-popup',
      placement: 'bottom'
    });
  }

  return {
    loaded: loaded
  };
}
