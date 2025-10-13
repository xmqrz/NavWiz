export default function (viz, grid, dataView) {
  var state = false;

  var headers = viz.models.headers();
  var navIdx = headers.indexOf('Navigation');
  var navXIdx = headers.indexOf('NavigationX');

  if (navIdx >= 0 || navXIdx >= 0) {
    const origGetItemMetadata = dataView.getItemMetadata.bind(dataView);
    dataView.getItemMetadata = getItemMetadata(origGetItemMetadata);
  }

  function getItemMetadata(origGetItemMetadata) {
    return function (row) {
      if (!state) {
        return origGetItemMetadata(row);
      }

      let item = dataView.getItem(row);
      let metadata = origGetItemMetadata(row);
      if (!item) {
        return metadata;
      }

      metadata = metadata || {};
      metadata.cssClasses = metadata.cssClasses || '';

      let navigation = item[navXIdx];
      if (!navigation || navigation === 'Idle') {
        navigation = item[navIdx];
      }

      if (!navigation) {
        return metadata;
      }

      if (navigation.indexOf('Forward') !== -1) {
        metadata.cssClasses += ' nav-forward';
      } else if (navigation.indexOf('Reverse') !== -1) {
        metadata.cssClasses += ' nav-reverse';
      } else if (navigation.indexOf('left') !== -1) {
        metadata.cssClasses += ' nav-left';
      } else if (navigation.indexOf('right') !== -1) {
        metadata.cssClasses += ' nav-right';
      } else if (navigation !== 'Idle') {
        metadata.cssClasses += ' nav-misc';
      }

      return metadata;
    };
  }

  function setState(newState) {
    state = newState;
    grid.invalidate();
  }

  return {
    setState: setState
  };
}
