/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import $ from 'cash-dom';
import * as _ from 'lodash-es';

// SlickGrid expect it exist globally.
import Sortable from 'sortablejs';
window.Sortable = Sortable;

import {
  SlickGrid,
  SlickCellSelectionModel,
  SlickAutoTooltips,
  SlickDataView
} from 'slickgrid';
import FilterPlugin from './filter-plugin';
import navActionHintF from './nav-action-hint';

var tableHtml = `
<div class="diagnostic-table-view-table" style="flex-grow:1;">
</div>
`;

export default function (viz) {
  var table = $(tableHtml);
  viz.append(table);

  var grid;
  var dataView = new SlickDataView();
  var headers;

  function loaded() {
    let data = viz.models.getData([]);
    let h = viz.models.headers();
    headers = h.map((v, i) => {
      return {
        id: i.toString(),
        name: v,
        field: i,
        width: i === 0 ? 150 : 200
      };
    });
    var options = {
      editable: false,
      enableAddRow: false,
      enableCellNavigation: true,
      asyncEditorLoading: false,
      rowHeight: 30,
      selectedCellCssClass: 'selected',
      frozenColumn: 0
    };
    let rows = data.map((d, idx) => {
      let r = {
        id: idx
      };
      for (let i in d) {
        r[i.toString()] = d[i];
      }
      return r;
    });
    dataView.setItems(rows);

    grid = new SlickGrid(table[0], dataView, headers, options);
    grid.setSelectionModel(
      new SlickCellSelectionModel({
        selectActiveCell: true
      })
    );
    grid.registerPlugin(new SlickAutoTooltips());

    initPlugins();
    initHints();
  }

  function setColumns(columns) {
    let newHeaders;
    if (!columns) {
      newHeaders = [headers.find((h) => h.id === '0')];
    } else {
      newHeaders = headers.filter((h) => h.id === '0' || columns.indexOf(h.id) >= 0);
    }

    grid.setColumns(newHeaders);
  }

  function scrollToColumn(columnId) {
    let columnIdx = grid.getColumnIndex(columnId);
    grid.scrollColumnIntoView(columnIdx);
  }

  function resized() {
    if (grid) {
      grid.resizeCanvas();
    }
  }

  function close() {
    if (grid) {
      grid.destroy();
    }
  }

  // filter plugin
  function initPlugins() {
    grid.registerPlugin(new FilterPlugin());
  }

  // hint layer
  var navActionHint;

  function initHints() {
    navActionHint = navActionHintF(viz, grid, dataView);
  }

  return {
    loaded: loaded,
    setColumns: setColumns,
    scrollToColumn: scrollToColumn,
    resized: resized,
    close: close,

    setNavHint: (s) => navActionHint.setState(s)
  };
}
