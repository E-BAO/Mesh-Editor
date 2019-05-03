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
      vector<Vector2D> lastLevel =  evaluatedLevels.back();
      if(lastLevel.size() < 2)
          return;
      vector<Vector2D> newLevel;
      for(int i = 1; i < lastLevel.size(); i ++){
          newLevel.push_back(t * lastLevel[i] + (1-t)*lastLevel[i - 1]);
      }
      evaluatedLevels.push_back(newLevel);
      return;
  }


  Vector3D BezierPatch::evaluate(double u, double v) const
  {
    // TODO Part 2.
    // Evaluate the Bezier surface at parameters (u, v) through 2D de Casteljau subdivision.
    // (i.e. Unlike Part 1 where we performed one subdivision level per call to evaluateStep, this function
    // should apply de Casteljau's algorithm until it computes the final, evaluated point on the surface)
      vector<Vector3D> newControlPoints;
      for(int i = 0; i  < controlPoints.size(); i ++){
          newControlPoints.push_back(evaluate1D(controlPoints[i], u));
      }
      return evaluate1D(newControlPoints, v);
  }

  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> points, double t) const
  {
    // TODO Part 2.
    // Optional helper function that you might find useful to implement as an abstraction when implementing BezierPatch::evaluate.==
    // Given an array of 4 points that lie on a single curve, evaluates the Bezier curve at parameter t using 1D de Casteljau subdivision.
      vector<Vector3D> lastLevel = points;
      vector<Vector3D> nextLevel;
      while(lastLevel.size() > 1){
          for(int i = 1; i < lastLevel.size(); i ++){
              nextLevel.push_back(t * lastLevel[i] + (1-t) * lastLevel[i - 1]);
          }
          lastLevel = nextLevel;
          nextLevel.clear();
      }
      
      return lastLevel.front();
 }



  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // TODO Returns an approximate unit normal at this vertex, computed by
    // TODO taking the area-weighted average of the normals of neighboring
    // TODO triangles, then normalizing.
      
      Vector3D n(0,0,0); // initialize a vector to store your normal sum
      HalfedgeCIter h = halfedge();
      
      HalfedgeCIter h_orig = h;
      
      VertexCIter center = h->vertex();
      HalfedgeCIter h_twin = h->twin();
      VertexCIter p1 = h_twin->vertex();
      VertexCIter p2;
      Vector3D v_edge1 = p1->position - center->position;
      Vector3D v_edge2;
      //S = |ABxAC|/2
      do{
          h_twin = h->twin();
          h = h_twin->next();
          p2 = h_twin->vertex();
          v_edge2 = p2->position - center->position;
          n += cross(v_edge2, v_edge1);
          v_edge1 = v_edge2;
      }while (h != h_orig);
      return n.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // TODO This method should flip the given edge and return an iterator to the flipped edge.
      if(e0->isBoundary())
          return e0;
      
      HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h0->twin();
      HalfedgeIter h4 = h3->next();
      HalfedgeIter h5 = h4->next();
      HalfedgeIter h6 = h1->twin();
      HalfedgeIter h7 = h2->twin();
      HalfedgeIter h8 = h4->twin();
      HalfedgeIter h9 = h5->twin();
      
      VertexIter v0 = h0->vertex();
      VertexIter v1 = h3->vertex();
      VertexIter v2 = h2->vertex();
      VertexIter v3 = h5->vertex();

      EdgeIter e1 = h1->edge();
      EdgeIter e2 = h2->edge();
      EdgeIter e3 = h4->edge();
      EdgeIter e4 = h5->edge();
      
      FaceIter f0 = h0->face();
      FaceIter f1 = h3->face();
      
      h0->setNeighbors(h1, h3, v3, e0, f0);
      h1->setNeighbors(h2, h7, v2, e2, f0);
      h2->setNeighbors(h0, h8, v0, e3, f0);
      h3->setNeighbors(h4, h0, v2, e0, f1);
      h4->setNeighbors(h5, h9, v3, e4, f1);
      h5->setNeighbors(h3, h6, v1, e1, f1);
      h6->setNeighbors(h6->next(), h5, v2, e1, h6->face());
      h7->setNeighbors(h7->next(), h1, v0, e2, h7->face());
      h8->setNeighbors(h8->next(), h2, v3, e3, h8->face());
      h9->setNeighbors(h9->next(), h4, v1, e4, h9->face());
      
      v0->halfedge() = h2;
      v1->halfedge() = h5;
      v2->halfedge() = h3;
      v3->halfedge() = h0;
      
      e0->halfedge() = h0;
      e1->halfedge() = h5;
      e2->halfedge() = h1;
      e3->halfedge() = h2;
      e4->halfedge() = h4;
      
      f0->halfedge() = h0;
      f1->halfedge() = h3;
      return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // TODO This method should split the given edge and return an iterator to the newly inserted vertex.
    // TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.
      if(e0->isBoundary())
          return (e0->halfedge())->vertex();
      
      HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h0->twin();
      HalfedgeIter h4 = h3->next();
      HalfedgeIter h5 = h4->next();
      HalfedgeIter h6 = h1->twin();
      HalfedgeIter h7 = h2->twin();
      HalfedgeIter h8 = h4->twin();
      HalfedgeIter h9 = h5->twin();
      
      VertexIter v0 = h0->vertex();
      VertexIter v1 = h3->vertex();
      VertexIter v2 = h2->vertex();
      VertexIter v3 = h5->vertex();
      
      EdgeIter e1 = h1->edge();
      EdgeIter e2 = h2->edge();
      EdgeIter e3 = h4->edge();
      EdgeIter e4 = h5->edge();
      EdgeIter e5 = newEdge();
      EdgeIter e6 = newEdge();
      EdgeIter e7 = newEdge();

      e5->isNew = false;
      e6->isNew = true;
      e7->isNew = true;
      
      FaceIter f0 = h0->face();
      FaceIter f1 = h3->face();
      FaceIter f2 = newFace();
      FaceIter f3 = newFace();
      
      VertexIter center = newVertex();
      center->position = (v1->position + v0->position)*.5;
      center->isNew = true;
      
      HalfedgeIter h10 = newHalfedge();
      HalfedgeIter h11 = newHalfedge();
      HalfedgeIter h12 = newHalfedge();
      HalfedgeIter h13 = newHalfedge();
      HalfedgeIter h14 = newHalfedge();
      HalfedgeIter h15 = newHalfedge();

      h0->setNeighbors(h5, h3, center, e0, f0);
      h1->setNeighbors(h12, h7, v2, e2, f3);
      h2->setNeighbors(h14, h8, v0, e3, f2);
      h3->setNeighbors(h10, h0, v1, e0, f1);
      h4->setNeighbors(h3, h9, v3, e4, f1);
      h5->setNeighbors(h11, h6, v1, e1, f0);
      h6->setNeighbors(h6->next(), h5, v2, e1, h6->face());
      h7->setNeighbors(h7->next(), h1, v0, e2, h7->face());
      h8->setNeighbors(h8->next(), h2, v3, e3, h8->face());
      h9->setNeighbors(h9->next(), h4, v1, e4, h9->face());
      h10->setNeighbors(h4, h14, center, e6, f1);
      h11->setNeighbors(h0, h15, v2, e7, f0);
      h12->setNeighbors(h15, h13, v0, e5, f3);
      h13->setNeighbors(h2, h12, center, e5, f2);
      h14->setNeighbors(h13, h10, v3, e6, f2);
      h15->setNeighbors(h1, h11, center, e7, f3);

      v0->halfedge() = h2;
      v1->halfedge() = h5;
      v2->halfedge() = h1;
      v3->halfedge() = h4;
      center->halfedge() = h0;
      
      e0->halfedge() = h0;
      e1->halfedge() = h5;
      e2->halfedge() = h1;
      e3->halfedge() = h2;
      e4->halfedge() = h4;
      e5->halfedge() = h13;
      e6->halfedge() = h10;
      e7->halfedge() = h11;
      
      f0->halfedge() = h0;
      f1->halfedge() = h4;
      f2->halfedge() = h2;
      f3->halfedge() = h1;
      
      return center;
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
      
      for(VertexIter it = mesh.verticesBegin(); it!=mesh.verticesEnd(); it++){
          it->isNew = false;
          int count_nb = 0;
          Vector3D nb_position_sum = 0.0;
          HalfedgeIter h = it->halfedge();
          HalfedgeIter h_twin;
          VertexIter nb;
          do{
              h_twin = h->twin();
              h = h_twin->next();
              nb_position_sum += (h_twin->vertex())->position;
              count_nb ++;
          }while (h != it->halfedge());
          double u = count_nb == 3? 3.0/16.0 : 3.0 / (double)(8*count_nb);
          it->newPosition = (1.0 - (double)count_nb * u) * it->position + u * nb_position_sum;
      }

    // TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
      
      for(EdgeIter it = mesh.edgesBegin(); it != mesh.edgesEnd(); it++){
          it->isNew = false;
          HalfedgeIter h = it->halfedge();
          HalfedgeIter h_twin = h->twin();
          VertexIter A = h->vertex();
          VertexIter B = h_twin->vertex();
          h = (h->next())->next();
          VertexIter D = h->vertex();
          h_twin = (h_twin->next())->next();
          VertexIter C = h_twin->vertex();
          it->newPosition = 0.375 * (A->position + B->position) +
                            0.125 * (C->position + D->position);
      }


    // TODO Next, we're going to split every edge in the mesh, in any order.  For future
    // TODO reference, we're also going to store some information about which subdivided
    // TODO edges come from splitting an edge in the original mesh, and which edges are new,
    // TODO by setting the flat Edge::isNew.  Note that in this loop, we only want to iterate
    // TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
    // TODO just split (and the loop will never end!)
      
      EdgeIter it_end = mesh.edgesEnd();
      it_end --;
      EdgeIter it = mesh.edgesBegin();
      while(1){
          if(!it->isNew){
              VertexIter v_it = mesh.splitEdge(it);
              v_it -> newPosition = it->newPosition;
          }
          
          if(it == it_end)
              break;
          
          it ++;
      }

      it_end ++;
    // TODO Now flip any new edge that connects an old and new vertex.
      for(it = it_end;it != mesh.edgesEnd(); it++){
          if(it->isNew){
              HalfedgeCIter h = it->halfedge();
              if(((h->vertex())->isNew && !((h->next())->vertex())->isNew) || \
                 (!(h->vertex())->isNew && ((h->next())->vertex())->isNew)){
                  mesh.flipEdge(it);
              }
          }
      }

    // TODO Finally, copy the new vertex positions into final Vertex::position.
      for(VertexIter it = mesh.verticesBegin(); it != mesh.verticesEnd(); it++){
//          if(it->isNew)
            it->position = it->newPosition;
      }
      
    return;
  }
}
