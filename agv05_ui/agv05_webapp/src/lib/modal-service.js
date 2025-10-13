export const alertModal = function (title, body, buttonTextCancel = 'Noted') {
  return {
    type: 'alert',
    title: title,
    body: body,
    position: 'items-center',
    buttonTextCancel: buttonTextCancel
  };
};

export const httpErrorModal = function (title, http = null) {
  return {
    type: 'component',
    component: 'modalShowHttpError',
    position: 'items-center',
    title: title,
    meta: { http: http }
  };
};
