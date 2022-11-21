#include "state.h"
#include "state_math.h"
#include "stdlib.h"
#include "stdio.h"
#include <tgmath.h>

state_t* state_new(size_t many)
{
	return(state_t *)calloc(many, sizeof(state_t));
}

record_t* record_new(size_t many)
{
	return (record_t *)calloc(many, sizeof(record_t));
}
series_t* series_new(size_t many)
{
	return (series_t *)calloc(many, sizeof(series_t));
}

state_t state_zero(void)
{
	state_t State;
	State.phi = 0;
	State.theta = 0;
	State.psi = 0;
	State.dot_phi = 0;
	State.dot_theta = 0;
	State.dot_psi = 0;
	State.pn = 0;
	State.pe = 0;
	State.pu = 0;
	State.vn = 0;
	State.ve = 0;
	State.vu = 0;
	State.fx = 0;
	State.fy = 0;
	State.fz = 0;
	State.P = 0;
	State.Q = 0;
	State.R = 0;
	return State;
}

void state_set_input(state_t *State, imu_data_t *Data)
{
	State->fx = Data->Ax;
	State->fy = Data->Ay;
	State->fz = Data->Az;
	State->P  = Data->Gx;
	State->Q  = Data->Gy;
	State->R  = Data->Gz;
}

void state_set_state(state_t *State, state_t *Prev)
{
	State->phi		= Prev->phi;
	State->theta		= Prev->theta;
	State->psi		= Prev->psi;
	State->pn		= Prev->pn;
	State->pe		= Prev->pe;
	State->pu		= Prev->pu;
	State->dot_phi		= Prev->dot_phi;
	State->dot_theta	= Prev->dot_theta;
	State->dot_psi		= Prev->dot_psi;
	State->vn		= Prev->vn;
	State->ve		= Prev->ve;
	State->vu		= Prev->vu;
}

void state_init(state_t *State, state_t *Prev, imu_data_t *Data)
{
	state_set_state(State, Prev);
	state_set_input(State, Data);
}

state_t state_next(state_t *Prev, imu_data_t *Data)
{
	state_t State = *Prev;
	state_set_input(&State, Data);
	return State;
}

record_t record_zero(void)
{
	record_t Record;
	Record.prev = NULL;
	Record.next = NULL;
	Record.count = 0;
	Record.X = state_zero();
	return Record;
}

state_t record_next_state(record_t *Prev, imu_data_t *Data)
{
	return state_next(&Prev->X, Data);
}

void record_link(record_t *Next, record_t *Prev)
{
	Next->prev = Prev;
	Prev->next = Next;
	Next->count = Prev->count +1;
}

void record_next(record_t *Next, record_t *Prev, imu_data_t *Data)
{
	record_link(Next, Prev);
	Next->X = record_next_state(Next->prev, Data);
}

void record_init(record_t *Record, record_t *Prev, imu_data_t *Data)
{
	record_link(Record, Prev);
	state_init(&Record->X, &Prev->X, Data);
}

void record_push(series_t *Series, record_t *Record)
{
	record_next(Record, Series->last, &Series->imu->dat);
	Series->last = Record;
}

void series_init(series_t *Series,
		char * Description,
		imu_t * Imu,
		const integration_t * Integration,
		size_t Size)
{
	Series->first = NULL;
	Series->last = NULL;
	Series->pos = NULL;
	Series->size = Size;
	Series->imu = Imu;
	Series->integ = *Integration;
	snprintf(Series->description, MAXLINE, "%s", Description);
	/* create an array of records in the heap */
	Series->first = record_new(Size);
	Series->last = Series->first;
}



void series_destroy(series_t *Series){
	free(Series->first);
	free(Series);
}

void series_test(void)
{
	integration_t trapez100 = {
		.method = RK_TRAPEZ,
		.steps = 100
	};

//	position_t origin = { .p={ 0, 0, 0 }};
	position_t cm_pos = { .p={ 0, 0, 0 }};

	imu_t Imu = {
		.description = "MPU-6050",
		.dat = { 1,2,0,0,0,0 },
		.cfg = { 4, 400, 250 },
		.relpos = {
		       	.reference = &cm_pos,
			.pos  = { .p1=0, .p2=0, .p3=0 },
			.att  = { .phi= M_PI, .theta=0, .psi=0 }
		}
	};

//	state_t State = state_zero();
	series_t *Series = series_new(1);
	series_init(Series, "Series testing", &Imu, &trapez100, 10ul);

	/* most basic sutff */
	printf("\"%s\"[%zu] IMU:\"%s\" at %LfHz, Method:%i, %zu steps \n",
		       	Series->description,
		       	Series->size,
		       	Series->imu->description,
		       	Series->imu->cfg.Fs,
		       	Series->integ.method,
		       	Series->integ.steps
	      );
	printf("->first: %p\n", (void*)Series->first);
	printf("->last:  %p\n", (void*)Series->last);

	/* test pushing records */
	printf("pushing records..\n");
	printf("Series->first (%p) Series->last (%p)\n", (void*)Series->first, (void*)Series->last);
	for(size_t i = 1; i <  Series->size; i++) {
		record_push(Series, Series->first + i);
	}

	printf(".. done\n");
	printf("printing records..\n");
	printf("Series->first (%p) Series->last (%p)\n", (void*)Series->first, (void*)Series->last);
	for(record_t *i = Series->first; i <=  Series->last; i++) {
		printf("%zu (%p) i->prev (%p)  i->next (%p)\n", i->count, (void*)i, (void*)i->prev, (void*)i->next);
	}
}

data_t vec3_magnitude(vec3_t A)
{
	return sqrt(square(A.v1) + square(A.v2) + square(A.v3));

}

/* DOT product of two vectors at an angle
 * JOHNSON, 1.2-2
 */
data_t vec3_prod_dot(vec3_t A, vec3_t B, data_t angle)
{
	return vec3_magnitude(A) * vec3_magnitude(B) * cos(angle);
}

///* CROSS product */
//data_t vec3_projection(vec3_t U, vec3_t V, data_t angle)
//{
//	return vec3_prod_dot(U, V, angle) / vec3_magnitude(V);
//
//}

//vec3_t vec3_prod_cross(vec3_t A, vec3_t B, data_t angle)
//{
//	return 
//}
