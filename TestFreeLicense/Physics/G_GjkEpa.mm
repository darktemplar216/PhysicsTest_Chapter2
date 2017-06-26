/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2008 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the
use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not
claim that you wrote the original software. If you use this software in a
product, an acknowledgment in the product documentation would be appreciated
but is not required.
2. Altered source versions must be plainly marked as such, and must not be
misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/*
GJK-EPA collision solver by Nathanael Presson, 2008
*/

#include "G_GjkEpa.h"
#import <Foundation/Foundation.h>

	// Config

	/* GJK	*/ 
#define GJK_MAX_ITERATIONS	128

#ifdef BT_USE_DOUBLE_PRECISION
	#define GJK_ACCURACY		((float)1e-12)
	#define GJK_MIN_DISTANCE	((float)1e-12)
	#define GJK_DUPLICATED_EPS	((float)1e-12)
#else
	#define GJK_ACCURACY		((float)0.0001)
	#define GJK_MIN_DISTANCE	((float)0.0001)
	#define GJK_DUPLICATED_EPS	((float)0.0001)
#endif //BT_USE_DOUBLE_PRECISION


#define GJK_SIMPLEX2_EPS	((float)0.0)
#define GJK_SIMPLEX3_EPS	((float)0.0)
#define GJK_SIMPLEX4_EPS	((float)0.0)

	/* EPA	*/ 
#define EPA_MAX_VERTICES	128
#define EPA_MAX_ITERATIONS	255

#ifdef BT_USE_DOUBLE_PRECISION
	#define EPA_ACCURACY		((float)1e-12)
	#define EPA_PLANE_EPS		((float)1e-14)
	#define EPA_INSIDE_EPS		((float)1e-9)
#else
	#define EPA_ACCURACY		((float)0.0001)
	#define EPA_PLANE_EPS		((float)0.00001)
	#define EPA_INSIDE_EPS		((float)0.01)
#endif

#define EPA_FALLBACK            (10*EPA_ACCURACY)
#define EPA_MAX_FACES           (EPA_MAX_VERTICES*2)

	// MinkowskiDiff
	struct	MinkowskiDiff
	{
		const PointCloud* mCobjs[2];
		const Matrix4* mTrans[2];
		inline Vector3 Support0(const Vector3& d) const
		{
            Vector3 ret = mCobjs[0]->getFarthestVectAtDir(d, mTrans[0]);
            //NSLog(@"Support0: x -> %f, y -> %f, z -> %f", ret.x, ret.y, ret.z);
			return(ret);
		}
		inline Vector3 Support1(const Vector3& d) const
		{
            Vector3 ret = mCobjs[1]->getFarthestVectAtDir(d, mTrans[1]);
            //NSLog(@"Support1: x -> %f, y -> %f, z -> %f", ret.x, ret.y, ret.z);
			return(ret);
		}
		inline Vector3	Support(const Vector3& d) const
		{
			return(Support0(d) - Support1(-d));
		}
		Vector3 Support(const Vector3& d,unsigned int index) const
		{
			if(index)
				return(Support1(d));
			else
				return(Support0(d));
		}
	};

	typedef	MinkowskiDiff	tShape;

	static float dot(const Vector3& a, const Vector3 b)
	{
		return a.dot(b);
	}
	static Vector3 cross(const Vector3& a, const Vector3 b)
	{
        return a.cross(b);
	}

	// GJK
	struct	GJK
	{
		/* Types		*/ 
		struct	sSV
		{
			Vector3	d,w;
		};
		struct	sSimplex
		{
			sSV*		c[4];
			float	p[4];
			unsigned int			rank;
		};
		struct	eStatus	{ enum _ {
			Valid,
			Inside,
			Failed		};};
			/* Fields		*/ 
			tShape			m_shape;
			Vector3		m_ray;
			float		m_distance;
			sSimplex		m_simplices[2];
			sSV				m_store[4];
			sSV*			m_free[4];
			unsigned int				m_nfree;
			unsigned int				m_current;
			sSimplex*		m_simplex;
			eStatus::_		m_status;
			/* Methods		*/ 
			GJK()
			{
				Initialize();
			}
			void				Initialize()
			{
				m_ray		=	Vector3(0,0,0);
				m_nfree		=	0;
				m_status	=	eStatus::Failed;
				m_current	=	0;
				m_distance	=	0;
			}
			eStatus::_			Evaluate(const tShape& shapearg,const Vector3& guess)
			{
				unsigned int			iterations=0;
				float	sqdist=0;
				float	alpha=0;
				Vector3	lastw[4];
				unsigned int			clastw=0;
				/* Initialize solver		*/ 
				m_free[0]			=	&m_store[0];
				m_free[1]			=	&m_store[1];
				m_free[2]			=	&m_store[2];
				m_free[3]			=	&m_store[3];
				m_nfree				=	4;
				m_current			=	0;
				m_status			=	eStatus::Valid;
				m_shape				=	shapearg;
				m_distance			=	0;
				/* Initialize simplex		*/ 
				m_simplices[0].rank	=	0;
				m_ray				=	guess;
                const float	sqrl    =	m_ray.lengthSquared();
				appendvertice(m_simplices[0], sqrl>0 ? -m_ray : Vector3(1,0,0));
				m_simplices[0].p[0]	=	1;
				m_ray				=	m_simplices[0].c[0]->w;	
				sqdist				=	sqrl;
				lastw[0]			=
					lastw[1]			=
					lastw[2]			=
					lastw[3]			=	m_ray;
				/* Loop						*/ 
				do	{
					const unsigned int		next=1-m_current;
					sSimplex&	cs=m_simplices[m_current];
					sSimplex&	ns=m_simplices[next];
					/* Check zero							*/ 
                    const float	rl= m_ray.length();
					if(rl<GJK_MIN_DISTANCE)
					{/* Touching or inside				*/ 
						m_status=eStatus::Inside;
						break;
					}
					/* Append new vertice in -'v' direction	*/ 
					appendvertice(cs, -m_ray);
					const Vector3&	w=cs.c[cs.rank-1]->w;
					bool				found=false;
					for(unsigned int i=0;i<4;++i)
					{
						if( (w - lastw[i]).lengthSquared() < GJK_DUPLICATED_EPS)
						{ found=true;break; }
					}
					if(found)
					{/* Return old simplex				*/ 
						removevertice(m_simplices[m_current]);
						break;
					}
					else
					{/* Update lastw					*/ 
						lastw[clastw=(clastw+1)&3]=w;
					}
					/* Check for termination				*/ 
					const float	omega=dot(m_ray,w)/rl;
					alpha=btMax(omega,alpha);
					if(((rl-alpha)-(GJK_ACCURACY*rl))<=0)
					{/* Return old simplex				*/ 
						removevertice(m_simplices[m_current]);
						break;
					}		
					/* Reduce simplex						*/ 
					float	weights[4];
					unsigned int			mask=0;
					switch(cs.rank)
					{
					case	2:	sqdist=projectorigin(	cs.c[0]->w,
									cs.c[1]->w,
									weights,mask);break;
					case	3:	sqdist=projectorigin(	cs.c[0]->w,
									cs.c[1]->w,
									cs.c[2]->w,
									weights,mask);break;
					case	4:	sqdist=projectorigin(	cs.c[0]->w,
									cs.c[1]->w,
									cs.c[2]->w,
									cs.c[3]->w,
									weights,mask);break;
					}
					if(sqdist>=0)
					{/* Valid	*/ 
						ns.rank		=	0;
						m_ray		=	Vector3(0,0,0);
						m_current	=	next;
						for(unsigned int i=0,ni=cs.rank;i<ni;++i)
						{
							if(mask&(1<<i))
							{
								ns.c[ns.rank]		=	cs.c[i];
								ns.p[ns.rank++]		=	weights[i];
								m_ray				=   m_ray + (cs.c[i]->w * weights[i]);
							}
							else
							{
								m_free[m_nfree++]	=	cs.c[i];
							}
						}
						if(mask==15)
                            m_status=eStatus::Inside;
					}
					else
					{/* Return old simplex				*/ 
						removevertice(m_simplices[m_current]);
						break;
					}
					m_status=((++iterations)<GJK_MAX_ITERATIONS)?m_status:eStatus::Failed;
				} while(m_status==eStatus::Valid);
				m_simplex=&m_simplices[m_current];
				switch(m_status)
				{
                    case eStatus::Valid:
                    {
                        m_distance= m_ray.length();
                        break;
                    }
                    case eStatus::Inside:
                    {
                        m_distance=0;
                        break;
                    }
                    default:
                        break;
				}	
				return(m_status);
			}
			bool					EncloseOrigin()
			{
				switch(m_simplex->rank)
				{
				case	1:
					{
						for(unsigned int i=0;i<3;++i)
						{
							Vector3		axis=Vector3(0,0,0);
							axis[i]=1;
							appendvertice(*m_simplex, axis);
							if(EncloseOrigin())	return(true);
							removevertice(*m_simplex);
							appendvertice(*m_simplex, -axis);
							if(EncloseOrigin())	return(true);
							removevertice(*m_simplex);
						}
					}
					break;
				case	2:
					{
						const Vector3	d= m_simplex->c[1]->w - m_simplex->c[0]->w;
						for(unsigned int i=0;i<3;++i)
						{
							Vector3		axis(0,0,0);
							axis[i]=1;
							const Vector3	p=cross(d,axis);
							if(p.lengthSquared()>0)
							{
								appendvertice(*m_simplex, p);
								if(EncloseOrigin())	return(true);
								removevertice(*m_simplex);
								appendvertice(*m_simplex, -p);
								if(EncloseOrigin())	return(true);
								removevertice(*m_simplex);
							}
						}
					}
					break;
				case	3:
					{
						const Vector3	n=cross(m_simplex->c[1]->w - m_simplex->c[0]->w, m_simplex->c[2]->w - m_simplex->c[0]->w);
						if( n.lengthSquared()>0)
						{
							appendvertice(*m_simplex,n);
							if(EncloseOrigin())	return(true);
							removevertice(*m_simplex);
							appendvertice(*m_simplex,-n);
							if(EncloseOrigin())	return(true);
							removevertice(*m_simplex);
						}
					}
					break;
				case	4:
					{
						if(btFabs(det(m_simplex->c[0]->w - m_simplex->c[3]->w,
                                      m_simplex->c[1]->w - m_simplex->c[3]->w,
                                      m_simplex->c[2]->w - m_simplex->c[3]->w))>0)
							return(true);
					}
					break;
				}
				return(false);
			}
			/* Internals	*/ 
			void				getsupport(const Vector3& d,sSV& sv) const
			{
				sv.d	=	d / d.length();
				sv.w	=	m_shape.Support(sv.d);
			}
			void				removevertice(sSimplex& simplex)
			{
				m_free[m_nfree++]=simplex.c[--simplex.rank];
			}
			void				appendvertice(sSimplex& simplex,const Vector3& v)
			{
				simplex.p[simplex.rank]=0;
				simplex.c[simplex.rank]=m_free[--m_nfree];
				getsupport(v,*simplex.c[simplex.rank++]);
			}
			static float		det(const Vector3& a,const Vector3& b,const Vector3& c)
			{
				return(	a.y*b.z*c.x+a.z*b.x*c.y-
					a.x*b.z*c.y-a.y*b.x*c.z+
					a.x*b.y*c.z-a.z*b.y*c.x);
			}
			static float		projectorigin(	const Vector3& a,
				const Vector3& b,
				float* w,unsigned int& m)
			{
				const Vector3	d= b - a;
				const float	l= d.lengthSquared();
				if(l>GJK_SIMPLEX2_EPS)
				{
					const float	t(l>0?-dot(a,d)/l:0);
					if(t>=1)		{ w[0]=0;w[1]=1;m=2;return(b.lengthSquared()); }
					else if(t<=0)	{ w[0]=1;w[1]=0;m=1;return(a.lengthSquared()); }
					else
                    {
                        w[0]=1-(w[1]=t);
                        m=3;
                        return(a + d * t).lengthSquared();
                    }
				}
				return(-1);
			}
			static float		projectorigin(	const Vector3& a,
				const Vector3& b,
				const Vector3& c,
				float* w,unsigned int& m)
			{
				static const unsigned int		imd3[]={1,2,0};
				const Vector3*	vt[]={&a,&b,&c};
				const Vector3		dl[]={a - b, b - c, c - a};
				const Vector3		n=cross(dl[0],dl[1]);
				const float		l= n.lengthSquared();
				if(l>GJK_SIMPLEX3_EPS)
				{
					float	mindist=-1;
					float	subw[2];
					unsigned int			subm;
					for(unsigned int i=0;i<3;++i)
					{
						if(dot(*vt[i],cross(dl[i],n))>0)
						{
							const unsigned int			j=imd3[i];
							const float	subd(projectorigin(*vt[i],*vt[j],subw,subm));
							if((mindist<0)||(subd<mindist))
							{
								mindist		=	subd;
								m			=	static_cast<unsigned int>(((subm&1)?1<<i:0)+((subm&2)?1<<j:0));
								w[i]		=	subw[0];
								w[j]		=	subw[1];
								w[imd3[j]]	=	0;				
							}
						}
					}
					if(mindist<0)
					{
						const float	d=dot(a,n);	
						const float	s=btSqrt(l);
						const Vector3	p= n * (d/l);
						mindist	=	p.lengthSquared();
						m		=	7;
						w[0]	=	cross(dl[1], b - p ).length() / s;
						w[1]	=	cross(dl[2], c - p ).length() / s;
						w[2]	=	1-(w[0]+w[1]);
					}
					return(mindist);
				}
				return(-1);
			}
			static float		projectorigin(	const Vector3& a,
				const Vector3& b,
				const Vector3& c,
				const Vector3& d,
				float* w,unsigned int& m)
			{
				static const unsigned int		imd3[]={1,2,0};
				const Vector3*	vt[]={&a,&b,&c,&d};
				const Vector3		dl[]={a - d, b - d, c - d};
				const float		vl=det(dl[0],dl[1],dl[2]);
				const bool			ng=(vl * dot(a, cross(b - c, a - b)))<=0;
				if(ng&&(btFabs(vl)>GJK_SIMPLEX4_EPS))
				{
					float	mindist=-1;
					float	subw[3];
					unsigned int			subm;
					for(unsigned int i=0;i<3;++i)
					{
						const unsigned int			j=imd3[i];
						const float	s=vl*dot(d,cross(dl[i],dl[j]));
						if(s>0)
						{
							const float	subd=projectorigin(*vt[i],*vt[j],d,subw,subm);
							if((mindist<0)||(subd<mindist))
							{
								mindist		=	subd;
								m			=	static_cast<unsigned int>((subm&1?1<<i:0)+
									(subm&2?1<<j:0)+
									(subm&4?8:0));
								w[i]		=	subw[0];
								w[j]		=	subw[1];
								w[imd3[j]]	=	0;
								w[3]		=	subw[2];
							}
						}
					}
					if(mindist<0)
					{
						mindist	=	0;
						m		=	15;
						w[0]	=	det(c,b,d)/vl;
						w[1]	=	det(a,c,d)/vl;
						w[2]	=	det(b,a,d)/vl;
						w[3]	=	1-(w[0]+w[1]+w[2]);
					}
					return(mindist);
				}
				return(-1);
			}
	};

	// EPA
	struct	EPA
	{
		/* Types		*/ 
		typedef	GJK::sSV	sSV;
		struct	sFace
		{
			Vector3	n;
			float	d;
			float	p;
			sSV*		c[3];
			sFace*		f[3];
			sFace*		l[2];
			unsigned char			e[3];
			unsigned char			pass;
		};
		struct	sList
		{
			sFace*		root;
			unsigned int			count;
			sList() : root(0),count(0)	{}
		};
		struct	sHorizon
		{
			sFace*		cf;
			sFace*		ff;
			unsigned int			nf;
			sHorizon() : cf(0),ff(0),nf(0)	{}
		};
		struct	eStatus { enum _ {
			Valid,
			Touching,
			Degenerated,
			NonConvex,
			InvalidHull,		
			OutOfFaces,
			OutOfVertices,
			AccuraryReached,
			FallBack,
			Failed		};};
			/* Fields		*/ 
			eStatus::_		m_status;
			GJK::sSimplex	m_result;
			Vector3		m_normal;
			float		m_depth;
			sSV				m_sv_store[EPA_MAX_VERTICES];
			sFace			m_fc_store[EPA_MAX_FACES];
			unsigned int				m_nextsv;
			sList			m_hull;
			sList			m_stock;
			/* Methods		*/ 
			EPA()
			{
				Initialize();	
			}


			static inline void		bind(sFace* fa,unsigned int ea,sFace* fb,unsigned int eb)
			{
				fa->e[ea]=(unsigned char)eb;fa->f[ea]=fb;
				fb->e[eb]=(unsigned char)ea;fb->f[eb]=fa;
			}
			static inline void		append(sList& list,sFace* face)
			{
				face->l[0]	=	0;
				face->l[1]	=	list.root;
				if(list.root) list.root->l[0]=face;
				list.root	=	face;
				++list.count;
			}
			static inline void		remove(sList& list,sFace* face)
			{
				if(face->l[1]) face->l[1]->l[0]=face->l[0];
				if(face->l[0]) face->l[0]->l[1]=face->l[1];
				if(face==list.root) list.root=face->l[1];
				--list.count;
			}


			void				Initialize()
			{
				m_status	=	eStatus::Failed;
				m_normal	=	Vector3(0,0,0);
				m_depth		=	0;
				m_nextsv	=	0;
				for(unsigned int i=0;i<EPA_MAX_FACES;++i)
				{
					append(m_stock,&m_fc_store[EPA_MAX_FACES-i-1]);
				}
			}
			eStatus::_			Evaluate(GJK& gjk,const Vector3& guess)
			{
				GJK::sSimplex&	simplex=*gjk.m_simplex;
				if((simplex.rank>1)&&gjk.EncloseOrigin())
				{

					/* Clean up				*/ 
					while(m_hull.root)
					{
						sFace*	f = m_hull.root;
						remove(m_hull,f);
						append(m_stock,f);
					}
					m_status	=	eStatus::Valid;
					m_nextsv	=	0;
					/* Orient simplex		*/ 
					if(gjk.det( simplex.c[0]->w - simplex.c[3]->w,
						simplex.c[1]->w - simplex.c[3]->w,
						simplex.c[2]->w - simplex.c[3]->w)<0)
					{
						btSwap(simplex.c[0],simplex.c[1]);
						btSwap(simplex.p[0],simplex.p[1]);
					}
					/* Build initial hull	*/ 
					sFace*	tetra[]={newface(simplex.c[0],simplex.c[1],simplex.c[2],true),
						newface(simplex.c[1],simplex.c[0],simplex.c[3],true),
						newface(simplex.c[2],simplex.c[1],simplex.c[3],true),
						newface(simplex.c[0],simplex.c[2],simplex.c[3],true)};
					if(m_hull.count==4)
					{
						sFace*		best=findbest();
						sFace		outer=*best;
						unsigned int			pass=0;
						unsigned int			iterations=0;
						bind(tetra[0],0,tetra[1],0);
						bind(tetra[0],1,tetra[2],0);
						bind(tetra[0],2,tetra[3],0);
						bind(tetra[1],1,tetra[3],2);
						bind(tetra[1],2,tetra[2],1);
						bind(tetra[2],2,tetra[3],1);
						m_status=eStatus::Valid;
						for(;iterations<EPA_MAX_ITERATIONS;++iterations)
						{
							if(m_nextsv<EPA_MAX_VERTICES)
							{	
								sHorizon		horizon;
								sSV*			w=&m_sv_store[m_nextsv++];
								bool			valid=true;					
								best->pass	=	(unsigned char)(++pass);
								gjk.getsupport(best->n,*w);
								const float	wdist=dot(best->n,w->w)-best->d;
								if(wdist>EPA_ACCURACY)
								{
									for(unsigned int j=0;(j<3)&&valid;++j)
									{
										valid&=expand(	pass,w,
											best->f[j],best->e[j],
											horizon);
									}
									if(valid&&(horizon.nf>=3))
									{
										bind(horizon.cf,1,horizon.ff,2);
										remove(m_hull,best);
										append(m_stock,best);
										best=findbest();
										outer=*best;
									} else { m_status=eStatus::InvalidHull;break; }
								} else { m_status=eStatus::AccuraryReached;break; }
							} else { m_status=eStatus::OutOfVertices;break; }
						}
						const Vector3	projection= outer.n * outer.d;
						m_normal	=	outer.n;
						m_depth		=	outer.d;
						m_result.rank	=	3;
						m_result.c[0]	=	outer.c[0];
						m_result.c[1]	=	outer.c[1];
						m_result.c[2]	=	outer.c[2];
						m_result.p[0]	=	cross(	outer.c[1]->w - projection, outer.c[2]->w - projection).length();
						m_result.p[1]	=	cross(	outer.c[2]->w - projection, outer.c[0]->w - projection).length();
						m_result.p[2]	=	cross(	outer.c[0]->w - projection, outer.c[1]->w - projection).length();
						const float	sum=m_result.p[0]+m_result.p[1]+m_result.p[2];
						const float onDivSum = 1 / sum;
						m_result.p[0]	*=	onDivSum;
						m_result.p[1]	*=	onDivSum;
						m_result.p[2]	*=	onDivSum;
						return(m_status);
					}
				}
				/* Fallback		*/ 
				m_status	=	eStatus::FallBack;
				m_normal	=	-guess;
				const float	nl= m_normal.length();
				if(nl>0)
					m_normal	= m_normal / nl;
				else
					m_normal	= Vector3(1,0,0);
				m_depth	=	0;
				m_result.rank=1;
				m_result.c[0]=simplex.c[0];
				m_result.p[0]=1;	
				return(m_status);
			}
			bool getedgedist(sFace* face, sSV* a, sSV* b, float& dist)
			{
				const Vector3 ba = b->w - a->w;
				const Vector3 n_ab = cross(ba, face->n); // Outward facing edge normal direction, on triangle plane
				const float a_dot_nab = dot(a->w, n_ab); // Only care about the sign to determine inside/outside, so not normalization required

				if(a_dot_nab < 0)
				{
					// Outside of edge a->b

					const float ba_l2 = ba.lengthSquared();
					const float a_dot_ba = dot(a->w, ba);
					const float b_dot_ba = dot(b->w, ba);

					if(a_dot_ba > 0)
					{
						// Pick distance vertex a
						dist = a->w.length();
					}
					else if(b_dot_ba < 0)
					{
						// Pick distance vertex b
						dist = b->w.length();
					}
					else
					{
						// Pick distance to edge a->b
						const float a_dot_b = dot(a->w, b->w);
						dist = sqrt(btMax((a->w.lengthSquared() * b->w.lengthSquared() - a_dot_b * a_dot_b) / ba_l2, (float)0));
					}

					return true;
				}

				return false;
			}
			sFace*				newface(sSV* a,sSV* b,sSV* c,bool forced)
			{
				if(m_stock.root)
				{
					sFace*	face=m_stock.root;
					remove(m_stock,face);
					append(m_hull,face);
					face->pass	=	0;
					face->c[0]	=	a;
					face->c[1]	=	b;
					face->c[2]	=	c;
					face->n		=	cross(b->w-a->w,c->w-a->w);
					const float	l=face->n.length();
					const bool		v=l>EPA_ACCURACY;

					if(v)
					{
						if(!(getedgedist(face, a, b, face->d) ||
							 getedgedist(face, b, c, face->d) ||
							 getedgedist(face, c, a, face->d)))
						{
							// Origin projects to the interior of the triangle
							// Use distance to triangle plane
							face->d = dot(a->w, face->n) / l;
						}

						face->n /= l;
						if(forced || (face->d >= -EPA_PLANE_EPS))
						{
							return face;
						}
						else
							m_status=eStatus::NonConvex;
					}
					else
						m_status=eStatus::Degenerated;

					remove(m_hull, face);
					append(m_stock, face);
					return 0;

				}
				m_status = m_stock.root ? eStatus::OutOfVertices : eStatus::OutOfFaces;
				return 0;
			}
			sFace*				findbest()
			{
				sFace*		minf=m_hull.root;
				float	mind=minf->d*minf->d;
				for(sFace* f=minf->l[1];f;f=f->l[1])
				{
					const float	sqd=f->d*f->d;
					if(sqd<mind)
					{
						minf=f;
						mind=sqd;
					}
				}
				return(minf);
			}
			bool				expand(unsigned int pass,sSV* w,sFace* f,unsigned int e,sHorizon& horizon)
			{
				static const unsigned int	i1m3[]={1,2,0};
				static const unsigned int	i2m3[]={2,0,1};
				if(f->pass!=pass)
				{
					const unsigned int	e1=i1m3[e];
					if((dot(f->n,w->w)-f->d)< -EPA_PLANE_EPS)
					{
						sFace*	nf=newface(f->c[e1],f->c[e],w,false);
						if(nf)
						{
							bind(nf,0,f,e);
							if(horizon.cf) bind(horizon.cf,1,nf,2); else horizon.ff=nf;
							horizon.cf=nf;
							++horizon.nf;
							return(true);
						}
					}
					else
					{
						const unsigned int	e2=i2m3[e];
						f->pass		=	(unsigned char)pass;
						if(	expand(pass,w,f->f[e1],f->e[e1],horizon)&&
							expand(pass,w,f->f[e2],f->e[e2],horizon))
						{
							remove(m_hull,f);
							append(m_stock,f);
							return(true);
						}
					}
				}
				return(false);
			}

	};

	//
	static void	Initialize(	const PointCloud* shape0,
		const PointCloud* shape1,
		const Matrix4* trans0,
		const Matrix4* trans1,
		btGjkEpaSolver2::sResults& results,
		tShape& shape,
		bool withmargins)
	{
		/* Results		*/ 
		results.witnesses[0] = results.witnesses[1]	= Vector3(0,0,0);
		results.status = btGjkEpaSolver2::sResults::Separated;
		/* Shape		*/ 
		shape.mCobjs[0]	= shape0;
		shape.mCobjs[1]	= shape1;
		shape.mTrans[0]	= trans0;
		shape.mTrans[1]	= trans1;
	}

//
bool		btGjkEpaSolver2::Distance(const PointCloud*	shape0,
									  const PointCloud*	shape1,
									  const Matrix4* trans0,
									  const Matrix4* trans1,
									  const Vector3& guess,
									  sResults&	results)
{
	tShape			shape;
	Initialize(shape0, shape1, trans0, trans1, results, shape, false);
	GJK				gjk;
	GJK::eStatus::_	gjk_status=gjk.Evaluate(shape,guess);
	if(gjk_status==GJK::eStatus::Valid)
	{
		Vector3	w0(0,0,0);
		Vector3	w1(0,0,0);
		for(unsigned int i=0;i<gjk.m_simplex->rank;++i)
		{
			const float	p=gjk.m_simplex->p[i];
			w0 = w0 + (shape.Support( gjk.m_simplex->c[i]->d, 0) * p);
			w1 = w1 + (shape.Support(-gjk.m_simplex->c[i]->d, 1) * p);
		}
		results.witnesses[0]	=	w0;
		results.witnesses[1]	=	w1;
		results.normal			=	w0 - w1;
		results.distance		=	results.normal.length();
		results.normal			=	results.normal / (results.distance > GJK_MIN_DISTANCE ? results.distance : 1);
		return(true);
	}
	else
	{
		results.status	=	gjk_status==GJK::eStatus::Inside?
			sResults::Penetrating	:
		sResults::GJK_Failed	;
		return(false);
	}
}

//
bool	btGjkEpaSolver2::Penetration(const PointCloud*	shape0,
									 const PointCloud*	shape1,
									 const Matrix4* trans0,
									 const Matrix4* trans1,
									 const Vector3&	guess,
									 sResults& results,
									 bool usemargins)
{
	tShape			shape;
	Initialize(shape0, shape1, trans0, trans1, results, shape, usemargins);
	GJK				gjk;	
	GJK::eStatus::_	gjk_status=gjk.Evaluate(shape, -guess);
	switch(gjk_status)
	{
	case	GJK::eStatus::Inside:
		{
			EPA				epa;
			EPA::eStatus::_	epa_status=epa.Evaluate(gjk, -guess);
			if(epa_status!=EPA::eStatus::Failed)
			{
				Vector3	w0(0,0,0);
				for(unsigned int i=0;i<epa.m_result.rank;++i)
				{
					w0 = w0 + (shape.Support(epa.m_result.c[i]->d, 0) * epa.m_result.p[i]);
				}
				results.status			=	sResults::Penetrating;
				results.witnesses[0]    =	w0;
				results.witnesses[1]	=	w0 - (epa.m_normal * epa.m_depth);
				results.normal			=	-epa.m_normal;
				results.distance		=	-epa.m_depth;
				return(true);
			} else results.status=sResults::EPA_Failed;
		}
		break;
	case	GJK::eStatus::Failed:
		results.status=sResults::GJK_Failed;
		break;
    default:
        break;
	}
	return(false);
}

/*
float	btGjkEpaSolver2::SignedDistance(const GLKVector3& position,
											float margin,
											const RigidBody* shape0,
											const GLKMatrix4* trans0,
											sResults& results)
{
	tShape			shape;
	G_SphereShapeP shape1;
	shape1.setCollisionData(position, margin);

	Initialize(shape0, &shape1, trans0, null, results, shape, false);
	GJK				gjk;	
	GJK::eStatus::_	gjk_status=gjk.Evaluate(shape,GLKVector3(1,1,1));
	if(gjk_status==GJK::eStatus::Valid)
	{
		GLKVector3	w0=GLKVector3(0,0,0);
		GLKVector3	w1=GLKVector3(0,0,0);
		for(unsigned int i=0;i<gjk.m_simplex->rank;++i)
		{
			const float	p=gjk.m_simplex->p[i];
			w0+=shape.Support( gjk.m_simplex->c[i]->d,0)*p;
			w1+=shape.Support(-gjk.m_simplex->c[i]->d,1)*p;
		}
		results.witnesses[0]	=	w0;
		results.witnesses[1]	=	w1;
		const GLKVector3	delta=	results.witnesses[1]-
			results.witnesses[0];
// 		const float	margin=	shape0->getMarginNonVirtual()+
// 			shape1.getMarginNonVirtual();
// 		const float	length=	delta.length();	
// 		results.normal			=	delta/length;
// 		results.witnesses[0]	+=	results.normal*margin;
// 		return(length-margin);
		const float	length=	delta.length();	
		results.normal			=	delta/length;
		return(length);
	}
	else
	{
		if(gjk_status==GJK::eStatus::Inside)
		{
			if(Penetration(shape0,&shape1,trans0,null,gjk.m_ray,results))
			{
				const GLKVector3	delta=	results.witnesses[0]-
					results.witnesses[1];
				const float	length=	delta.length();
				if (length >= EPSILON_FLT)
					results.normal	=	delta/length;			
				return(-length);
			}
		}	
	}
	return(INFINIT_FLT);
}
*/

bool btGjkEpaSolver2::SignedDistance(const PointCloud* shape0,
									const PointCloud* shape1,
									const Matrix4* trans0,
									const Matrix4* trans1,
									const Vector3& guess,
									sResults& results)
{
	if(!Distance(shape0, shape1, trans0, trans1, guess, results))
		return(Penetration(shape0, shape1, trans0, trans1, guess, results, false));
	else
		return(true);
}

/* Symbols cleanup		*/ 

#undef GJK_MAX_ITERATIONS
#undef GJK_ACCURARY
#undef GJK_MIN_DISTANCE
#undef GJK_DUPLICATED_EPS
#undef GJK_SIMPLEX2_EPS
#undef GJK_SIMPLEX3_EPS
#undef GJK_SIMPLEX4_EPS

#undef EPA_MAX_VERTICES
#undef EPA_MAX_FACES
#undef EPA_MAX_ITERATIONS
#undef EPA_ACCURACY
#undef EPA_FALLBACK
#undef EPA_PLANE_EPS
#undef EPA_INSIDE_EPS
