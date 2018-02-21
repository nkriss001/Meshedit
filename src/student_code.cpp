#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{
  void BezierCurve::evaluateStep()
  {
    // TODO Part 1.
    // Perform one step of the Bezier curve's evaluation at t using de Casteljau's algorithm for subdivision.
    // Store all of the intermediate control points into the 2D vector evaluatedLevels.
    int levels = evaluatedLevels.size();
    if (levels == numControlPoints) {
      return;
    } else {
      std::vector<Vector2D> points = evaluatedLevels[levels-1];
      std::vector<Vector2D> newPoints;
      for (int i = 0; i < points.size() - 1; i++) {
        Vector2D p0 = points[i];
        Vector2D p1 = points[i+1];
        Vector2D newPoint = (1 - t)*p0 + t*p1;
        newPoints.push_back(newPoint);
      }
      evaluatedLevels.push_back(newPoints);
    }
  }


  Vector3D BezierPatch::evaluate(double u, double v) const
  {
    // TODO Part 2.
    // Evaluate the Bezier surface at parameters (u, v) through 2D de Casteljau subdivision.
    // (i.e. Unlike Part 1 where we performed one subdivision level per call to evaluateStep, this function
    // should apply de Casteljau's algorithm until it computes the final, evaluated point on the surface)
    std::vector<Vector3D> newControlPoints;
    for (int i = 0; i < controlPoints.size(); i++) {
      newControlPoints.push_back(evaluate1D(controlPoints[i], u));
    }
    return evaluate1D(newControlPoints, v);
  }

  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> points, double t) const
  {
    // TODO Part 2.
    // Optional helper function that you might find useful to implement as an abstraction when implementing BezierPatch::evaluate.
    // Given an array of 4 points that lie on a single curve, evaluates the Bezier curve at parameter t using 1D de Casteljau subdivision.
    while (points.size() > 1) {
      std::vector<Vector3D> newPoints;
      for (int i = 0; i < points.size() - 1; i++) {
        Vector3D p0 = points[i];
        Vector3D p1 = points[i+1];
        Vector3D newPoint = (1 - t)*p0 + t*p1;
        newPoints.push_back(newPoint);
      }
      points = newPoints;
    }
    return points[0];
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // TODO Returns an approximate unit normal at this vertex, computed by
    // TODO taking the area-weighted average of the normals of neighboring
    // TODO triangles, then normalizing.
    Vector3D n(0, 0, 0);
    HalfedgeCIter h = halfedge();
    h = h->twin();
    HalfedgeCIter h_orig = h;
    Vector3D a = position;
    do {
      Vector3D c = h->vertex()->position;
      h = h->next();
      Vector3D b = h->next()->vertex()->position;
      Vector3D edge1 = b - a;
      Vector3D edge2 = c - b;
      n += cross(edge1, edge2);
      h = h->twin();
    } while (h != h_orig);
    return n.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // TODO This method should flip the given edge and return an iterator to the flipped edge.
    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter hTwin0 = h0->twin();
    if (h0->isBoundary() or hTwin0->isBoundary()) {
      return e0;
    }
    VertexIter b = h0->vertex();
    VertexIter c = hTwin0->vertex();

    HalfedgeIter h1 = h0->next();
    HalfedgeIter hTwin1 = hTwin0->next();
    HalfedgeIter h2 = h1->next();
    HalfedgeIter hTwin2 = hTwin1->next();

    VertexIter a = h2->vertex();
    VertexIter d = hTwin2->vertex();
    FaceIter face = h2->face();
    FaceIter faceTwin = hTwin2->face();

    h0->next() = h2;
    h0->vertex() = d;
    h2->next() = hTwin1;
    hTwin1->next() = h0;
    hTwin1->face() = face;
    
    hTwin0->next() = hTwin2;
    hTwin0->vertex() = a;
    hTwin2->next() = h1;
    h1->next() = hTwin0;
    h1->face() = faceTwin;

    face->halfedge() = hTwin1;
    faceTwin->halfedge() = h1;
    b->halfedge() = hTwin1;
    c->halfedge() = h1;

    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // TODO This method should split the given edge and return an iterator to the newly inserted vertex.
    // TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter hTwin0 = h0->twin();
    if (h0->isBoundary() or hTwin0->isBoundary()) {
      return e0->halfedge()->vertex();
    }
    VertexIter b = h0->vertex();
    VertexIter c = hTwin0->vertex();

    HalfedgeIter h1 = h0->next();
    HalfedgeIter hTwin1 = hTwin0->next();
    HalfedgeIter h2 = h1->next();
    HalfedgeIter hTwin2 = hTwin1->next();

    VertexIter a = h2->vertex();
    VertexIter d = hTwin2->vertex();
    FaceIter face = h2->face();
    FaceIter faceTwin = hTwin2->face();

    HalfedgeIter eh1 = newHalfedge();
    HalfedgeIter ehTwin1 = newHalfedge();
    HalfedgeIter eh2 = newHalfedge();
    HalfedgeIter ehTwin2 = newHalfedge();
    HalfedgeIter eh3 = newHalfedge();
    HalfedgeIter ehTwin3 = newHalfedge();
    FaceIter newFace1 = newFace();
    FaceIter newFace2 = newFace();
    EdgeIter e1 = newEdge();
    EdgeIter e2 = newEdge();
    EdgeIter e3 = newEdge();
    e1->halfedge() = eh1;
    e1->isNew = true;
    e2->halfedge() = eh2;
    e2->isNew = false;
    e3->halfedge() = eh3;
    e3->isNew = true;
    newFace1->halfedge() = h1;
    newFace2->halfedge() = hTwin1;

    Vector3D midpoint = (c->position - b->position)/2 + b->position;
    VertexIter m = newVertex();
    m->position = midpoint;
    m->halfedge() = ehTwin1;

    eh1->twin() = ehTwin1;
    eh1->next() = h0;
    eh1->vertex() = a;
    eh1->edge() = e1;
    eh1->face() = newFace1;
    h0->vertex() = m;
    h0->face() = newFace1;
    h1->next() = eh1;
    h1->face() = newFace1;

    ehTwin1->twin() = eh1;
    ehTwin1->next() = h2;
    ehTwin1->vertex() = m;
    ehTwin1->edge() = e1;
    ehTwin1->face() = face;
    h2->next() = eh2;
    eh2->twin() = ehTwin2;
    eh2->next() = ehTwin1;
    eh2->vertex() = b;
    eh2->edge() = e2;
    eh2->face() = face;

    ehTwin2->twin() = eh2;
    ehTwin2->next() = hTwin1;
    ehTwin2->vertex() = m;
    ehTwin2->edge() = e2;
    ehTwin2->face() = newFace2;
    hTwin1->next() = eh3;
    hTwin1->face() = newFace2;
    eh3->twin() = ehTwin3;
    eh3->next() = ehTwin2;
    eh3->vertex() = d;
    eh3->edge() = e3;
    eh3->face() = newFace2;

    ehTwin3->twin() = eh3;
    ehTwin3->next() = hTwin2;
    ehTwin3->vertex() = m;
    ehTwin3->edge() = e3;
    ehTwin3->face() = faceTwin;
    hTwin0->next() = ehTwin3;

    face->halfedge() = h2;
    faceTwin->halfedge() = hTwin2;
    b->halfedge() = eh2;
    return m;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // Each vertex and edge of the original surface can be associated with a vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to *first* compute the new positions
    // using the connectity of the original (coarse) mesh; navigating this mesh will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse. We will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.


    // TODO Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // TODO and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // TODO a vertex of the original mesh.


    // TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.


    // TODO Next, we're going to split every edge in the mesh, in any order.  For future
    // TODO reference, we're also going to store some information about which subdivided
    // TODO edges come from splitting an edge in the original mesh, and which edges are new,
    // TODO by setting the flat Edge::isNew.  Note that in this loop, we only want to iterate
    // TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
    // TODO just split (and the loop will never end!)


    // TODO Now flip any new edge that connects an old and new vertex.


    // TODO Finally, copy the new vertex positions into final Vertex::position.
    VertexIter vertex = mesh.verticesBegin();
    do {
      vertex->isNew = false;
      Size n = 0;
      Vector3D position_sum(0, 0, 0);
      HalfedgeIter h = vertex->halfedge();
      h = h->twin();
      HalfedgeIter h_start = h;
      do {
        position_sum += h->vertex()->position;
        n += 1;
        h = h->next()->twin();
      } while (h != h_start);
      float u;
      if (n == 3) {
        u = 3.0/16.0;
      } else {
        u = 3.0/(8.0*n);
      }
      vertex->newPosition = (1 - n*u) * vertex->position + u * position_sum;
      vertex++;
    } while (vertex != mesh.verticesEnd());

    EdgeIter edge = mesh.edgesBegin();
    do {
      HalfedgeIter h = edge->halfedge();
      VertexIter a = h->vertex();
      HalfedgeIter hTwin = h->twin();
      VertexIter b = hTwin->vertex();
      HalfedgeIter h2 = h->next()->next();
      VertexIter c = h2->vertex();
      HalfedgeIter hTwin2 = hTwin->next()->next();
      VertexIter d = hTwin2->vertex();
      edge->newPosition = (3.0/8.0) * (a->position + b->position) + (1.0/8.0) * (c->position + d->position);
      edge++;
    } while (edge != mesh.edgesEnd());

    edge = mesh.edgesBegin();
    do {
      HalfedgeIter h = edge->halfedge();
      VertexIter a = h->vertex();
      HalfedgeIter hTwin = h->twin();
      VertexIter b = hTwin->vertex();
      if (!(a->isNew) && !(b->isNew)) {
        VertexIter newVertex = mesh.splitEdge(edge);
        if (newVertex != a && newVertex != b) {
          newVertex->isNew = true;
          newVertex->newPosition = edge->newPosition;
        }
      } else if (edge->isNew && ((a->isNew && !(b->isNew)) || (!(a->isNew) && b->isNew))) {
        mesh.flipEdge(edge);
      }
      edge++;
    } while (edge != mesh.edgesEnd());

    vertex = mesh.verticesBegin();
    do {
      vertex->position = vertex->newPosition;
      vertex++;
    } while (vertex != mesh.verticesEnd());
  }

}
