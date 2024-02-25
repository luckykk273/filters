#ifndef FILTERS_TEMPLATE_H_
#define FILTERS_TEMPLATE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
} TemplateConfigT;

typedef struct {
} TemplateStateT;

typedef struct {
} TemplateInputT;

typedef struct {
  TemplateStateT state;
  TemplateConfigT config;
} TemplateT;


void template_update(TemplateT *template, const TemplateInputT *input);

size_t template_memsize(void);

void template_init(TemplateT *template);

void template_set_config(TemplateT *template, const TemplateConfigT *template_config);

void template_get_config(TemplateT *template, TemplateConfigT *template_config);

void template_set_quat(TemplateT *template, const double *q);

void template_get_quat(TemplateT *template, double *q);

#ifdef __cplusplus
}
#endif

#endif  // FILTERS_TEMPLATE_H_