import d3ext from './d3-ext';
import { selection } from 'd3-selection';
import selection_attrs from 'd3-selection-multi/src/selection/attrs';
import selection_styles from 'd3-selection-multi/src/selection/styles';

export * from 'node_modules/d3';
export const track = d3ext.track;
export const hover = d3ext.hover;

selection.prototype.attrs = selection_attrs;
selection.prototype.styles = selection_styles;
