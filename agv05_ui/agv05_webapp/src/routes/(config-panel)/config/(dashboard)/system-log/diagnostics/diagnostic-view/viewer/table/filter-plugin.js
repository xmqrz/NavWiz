/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import $ from 'cash-dom';
import Select from 'components/Select.svelte';
import { SlickEventHandler } from 'slickgrid';
import { floating } from '$lib/utils';

const NULL_PLACEHOLDER = '{null}';

var dropdownHtml = `
<div class="relative float-right z-50">
  <button type="button" class="text-gray-500">
    <i class="fa-solid fa-caret-down"></i>
  </button>
</div>
`;

export default function FilterPlugin(options) {
  /* FilterPlugin Class */
  if (!(this instanceof FilterPlugin)) {
    return new FilterPlugin(options);
  }

  options = options || {};

  var columnFilters = {};

  var grid;
  var handler = new SlickEventHandler();

  this.pluginName = 'FilterPlugin';
  this.init = function (g) {
    grid = g;
    handler
      .subscribe(grid.onHeaderCellRendered, handleHeaderCellRendered)
      .subscribe(grid.onBeforeHeaderCellDestroy, handleBeforeHeaderCellDestroy);

    let dataView = grid.getData();
    dataView.setFilter(filter);

    // Ensure dataview update will render grid.
    dataView.onRowCountChanged.subscribe(function (_e, _args) {
      grid.updateRowCount();
      grid.render();
    });

    dataView.onRowsChanged.subscribe(function (e, args) {
      grid.invalidateRows(args.rows);
      grid.render();
    });

    // Force the grid to re-render the header now that the events are hooked up.
    grid.setColumns(grid.getColumns());
  };

  this.destroy = function () {};

  this.setOptions = function (newOptions) {
    options = $.extend(true, {}, options, newOptions);
  };

  function handleHeaderCellRendered(e, args) {
    let column = args.column; // the column data.

    if (column.name === 'Timestamp') {
      return;
    }

    let header = $(args.node);
    header.css('z-index', 2);
    let dropdown = $(dropdownHtml);
    header.prepend(dropdown);
    dropdown.find('button').on('click', function () {
      let options = getFilterOption(column.id);
      if (options.length <= 1) {
        if (header.rendered) {
          return;
        }
        header.rendered = true;
        let text = $(
          `<span class="bg-surface-100-800-token rounded-md p-5">${options.length === 0 ? 'No Filter Available' : 'Only 1 Value Available'}</span>`
        );
        header.append(text);
        floating(this, text[0]).show();
        return;
      }
      let target = $('<span class="w-0 h-0 m-0 p-0 pr-1 relative float-right"></span>');
      target.insertBefore(dropdown);
      let select = new Select({
        target: target[0],
        props: {
          options: options.map((o) => [o, o]),
          offset: 20,
          buttonClass: 'opacity-0 w-0 h-0 m-0 p-0',
          enableFilter: false,
          multiple: true
        }
      });
      if (columnFilters[column.id]) {
        select.value = Array.from(columnFilters[column.id]);
      } else {
        select.value = [...options];
      }
      select.show();
      select.$on('close', function () {
        select.$destroy();
        target.remove();
      });
      select.$on('change', function () {
        filterRow(header, options, select.value, column);
      });
    });
  }

  function handleBeforeHeaderCellDestroy(e, args) {
    let header = $(args.node);
    let selection = header.find('.diagnostic-select');
    selection.remove();
  }

  function filterRow(header, options, value, column) {
    let val = value;
    if (val && (val.length === options.length || val.length === 0)) {
      columnFilters[column.id] = undefined;
      header.removeClass('filter');
    } else if (val) {
      let newFilter = new Set(val);
      if (newFilter.has(NULL_PLACEHOLDER)) {
        newFilter.delete(NULL_PLACEHOLDER);
        newFilter.add(null);
      }
      columnFilters[column.id] = newFilter;
      header.addClass('filter');
    }
    let dataView = grid.getData();
    dataView.refresh();
  }

  function getFilterOption(columnId) {
    const MAX_OPTION = 50;
    let options = new Set();
    let dataView = grid.getData();
    let items = dataView.getItems();
    for (let r of items) {
      let val = r[columnId];
      if (val === undefined) {
        continue;
      }

      options.add(val);
      if (options.size > MAX_OPTION) {
        return [];
      }
    }

    return Array.from(options).sort();
  }

  function filter(item) {
    return Object.entries(columnFilters).every(([columnId, val]) => {
      if (columnId !== undefined && val && val.size > 0) {
        var c = grid.getColumns()[grid.getColumnIndex(columnId)];
        if (!val.has(item[c.field])) {
          return false;
        }
      }
      return true;
    });
  }
}
