//
// Created by ahmed on 08/06/25.
//

#include "Visualizer.h"

namespace geneticalgorithm {
  void
  Visualizer::drawTrajectory(const Chromosome& chr, const Color& color, bool simple) {
    for(auto& t : chr.result().trace->traj) {
      if(!simple)
        drawRocket(
          t.position().x * SCALE,
          t.position().y * SCALE,
          t.currentSC().angle,
          t.currentSC().power,
          color,
          2.f);
      else
        drawRocketSimple(
          t.position().x * SCALE,
          t.position().y * SCALE,
          t.currentSC().angle,
          t.currentSC().power,
          color,
          6.f);
    }
  }

  void
  Visualizer::eventCheck() {
    if(IsKeyPressed(KEY_Z)) { drawGeneration_ = !drawGeneration_; }

    static const int INCREMENT{ 10 };
    if(IsKeyPressedRepeat(KEY_PAGE_DOWN) || IsKeyPressed(KEY_PAGE_DOWN)) {
      gatrace_->changeGenerationCursor(-INCREMENT, false);
    }
    if(IsKeyPressedRepeat(KEY_PAGE_UP) || IsKeyPressed(KEY_PAGE_UP)) {
      gatrace_->changeGenerationCursor(+INCREMENT, false);
    }
    if(IsKeyPressedRepeat(KEY_LEFT) || IsKeyPressed(KEY_LEFT)) {
      gatrace_->changeGenerationCursor(-1);
    }
    if(IsKeyPressedRepeat(KEY_RIGHT) || IsKeyPressed(KEY_RIGHT)) {
      gatrace_->changeGenerationCursor(+1);
    }
    if(IsKeyPressedRepeat(KEY_DOWN) || IsKeyPressed(KEY_DOWN)) {
      gatrace_->changeChromosomeCursor(-1);
    }
    if(IsKeyPressedRepeat(KEY_UP) || IsKeyPressed(KEY_UP)) {
      gatrace_->changeChromosomeCursor(+1);
    }
    if(IsKeyPressed(KEY_S)) { this->screenshotEverything("screenshot", 10); }
  }

  void
  Visualizer::screenshotEverything(std::filesystem::path const& outDirectory,
    int increment) {
    assert(increment > 0);
    std::filesystem::create_directory(outDirectory);
    if(!std::filesystem::exists(outDirectory)) {
      std::cerr << "-- " << outDirectory << " : Error while creating directory!\n";
      return;
    }

    const auto genCursorSaved{ gatrace_->genCursor() };
    const auto chrCursorSaved{ gatrace_->chrCursor() };
    const std::string workingDirSaved{ GetWorkingDirectory() };
    const auto genMax{ gatrace_->getAllGenerations().size() };

    ChangeDirectory(outDirectory.c_str());
    std::cerr << "-- Changing working directory: " << GetWorkingDirectory() << "\n";
    std::cerr << "-- Starting screenshot" << std::endl;
    gatrace_->setChromosomeCursor(0);

    for(long genCursor{ 0 }; genCursor != genMax;) {
      gatrace_->setGenerationCursor(genCursor);
      this->render(false);
      this->render(false);
      WaitTime(0.1);

      std::stringstream filename;
      filename << std::setw(5) << std::setfill('0') << genCursor << ".png";
      TakeScreenshot(filename.str().c_str());

      if(genCursor == genMax - 1) { genCursor++; }
      else {
        genCursor += increment;
        genCursor = std::min((size_t)genCursor, genMax - 1);
      }
    }
    std::cerr << "-- Screenshot done" << std::endl;

    gatrace_->setGenerationCursor(genCursorSaved);
    gatrace_->setChromosomeCursor(chrCursorSaved);
    ChangeDirectory(workingDirSaved.c_str());
    std::cerr << "-- Changing working directory: " << GetWorkingDirectory() << "\n";
  }

  void
  Visualizer::render(bool wait) {
    BeginDrawing();
    ClearBackground(WHITE);

    // Draw land
    for(auto it{ gatrace_->land().points().begin() + 1 };
        it != gatrace_->land().points().end(); it++) {
      auto pit = it - 1;
      DrawLine(pit->x * SCALE, pit->y * SCALE, it->x * SCALE, it->y * SCALE, RED);
    }
    // Current generation
    DrawText(("Gen: " + std::to_string(gatrace_->genCursor())).c_str(),
             30,
             400,
             30,
             RED);

    if(drawGeneration_) {
      auto beg{ gatrace_->generation().chr.rbegin() };
      auto end{ gatrace_->generation().chr.rend() };
      for(size_t ichr{ gatrace_->generation().chr.size() - 1 }; ichr != -1UL; ichr--) {
        //for(auto p{ beg }; p != end; p++) {
        Color drawingColor;
        //auto idx{ &(*p) - &(gatrace_->generation[gen].chr[0]) };
        if(ichr < gatrace_->parameters().numElite) { drawingColor = GREEN; }
        else { drawingColor = ORANGE; }
        // Draw ship trajectory
        drawTrajectory(gatrace_->generation().chr[ichr],
                       drawingColor,
                       true);
      }
    }

    auto& chr{ gatrace_->chromosome() };
    // Draw the parents trajectory and chromosome
    // if(drawParents) {
    //   for(size_t p{ 0 }; p < 2; p++) {
    //     Color drawingColor{ p == 0 ? BLUE : SKYBLUE };
    //     if(chr.m.parent[p] != nullptr) {
    //       auto& t{ chr.m.parent[p]->res.m.trajectory };
    //       drawTrajectory(t.begin(), t.end(), drawingColor);
    //       drawChromosome(*chr.m.parent[p], drawingColor, 150, 500 + 50 * p);
    //     }
    //   }
    // }
    // Draw the selected pheno trajectory, chromosome, and score
    drawTrajectory(chr,
                   PINK);
    std::stringstream tempstream;
    const auto& r{ chr.result() };
    const auto& scr{ chr.result().trace->score };

    // tempstream //
    //   << std::setprecision(3) << std::setw(7) << r.score //
    //   << std::setprecision(3) << std::setw(4) << scr.distance.x //
    //   << std::setprecision(3) << std::setw(4) << scr.distance.y //
    //   << std::setprecision(3) << std::setw(4) << scr.projectedDistance //
    //   << std::setprecision(3) << std::setw(4) << scr.velocity.x //
    //   << std::setprecision(3) << std::setw(4) << scr.velocity.y //
    //   << std::setprecision(3) << std::setw(4) << scr.angle //
    //   << std::setprecision(3) << std::setw(4) << scr.duration //
    //   << std::setprecision(3) << std::setw(4) << scr.fuelLeft;

    auto phenoScoreString{
      "Pheno: " + std::to_string(gatrace_->chrCursor()) + " " + std::to_string(
        r.score) };
    DrawText(phenoScoreString.c_str(), 150, 400, 20, PINK);
    tempstream << chr.result().trace->score;
    DrawText(tempstream.str().c_str(), 150, 430, 20, PINK);
    tempstream.str("");
    tempstream << chr.result().trace->number;
    DrawText(tempstream.str().c_str(), 150, 460, 20, PINK);
    //drawChromosome(chr, PINK, 150, 450);

    EndDrawing();
    if(wait) { WaitTime(1.0 / 24.0); }
  }

  void
  Visualizer::show() {
    if(!gatrace_->parameters().trace) { return; }

    SetTraceLogLevel(LOG_WARNING);
    //SetTraceLogLevel(LOG_ALL);
    InitWindow(1366.f,
               700.f,
               "Mars lander"); // Initialize window and OpenGL context
    while(!WindowShouldClose()) {
      eventCheck();
      render();
    }
    CloseWindow();
  }
} // geneticalgorithm