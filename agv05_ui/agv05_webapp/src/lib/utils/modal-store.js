export function extendModalStore(modalStore) {
  // Extend modalStore for priorityTrigger.
  modalStore.priorityTrigger = (m) => {
    return modalStore.update((mStore) => {
      mStore.unshift(m);
      return mStore;
    });
  };

  const originalClose = modalStore.close;
  modalStore.close = (m) => {
    if (m === undefined) {
      return originalClose();
    }
    return modalStore.update((mStore) => {
      return mStore.filter((item) => item !== m);
    });
  };
}
